%% presim_RH_alpha.m — Receding Horizon QP로 오프라인 α*(t) 계산
%
% 방법: 매 타임스텝 k에서 앞 H_hor 스텝 구간의 소규모 QP를 풀고
%       첫 번째 값만 적용 (MPC 스타일 receding horizon)
%
%   greedy α_B  : H_hor = 1  (현재 스텝만 최적화 → 동일)
%   offline QP  : H_hor = N  (전체 구간 한 번에)
%   RH QP       : 1 < H_hor < N  (중간)
%
% 동결 궤적 가정: 참조 시뮬(null_method='alpha_B')에서
%   bp = B·qdot_p, bv = B·v_null 추출 후 사용

clc; clear; close all;
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

%% ══════════════════════════════════════════════════════
%  설정
% ══════════════════════════════════════════════════════
H_hor    = 20;     % 예측 구간 (타임스텝 수, Ts=0.1s → 2초 미리보기)
lambda_s = 0.05;   % 스무딩 강도 (인접 α 차이 패널티)

QP_BLK = 'concept3_simulink_RL/QP/QP_sys_bl';

%% ══════════════════════════════════════════════════════
%  공통 초기 조건
% ══════════════════════════════════════════════════════
loaded_data = load('concept3_for_sim.mat');
robot = loaded_data.robot;
robot.DataFormat = 'column';

port_num       = 10;
sat_docked_num = 6;

port_target = AB2port(port_num);
sat_docked  = AB2port(sat_docked_num);

[opa, mass] = set_obs([0 0 0 0 0 0 0 0 1 0]);
obs_vec = opa;
obs_vec(sat_docked_num) = 1;

port = port_pos();

q0b   = [1;0;0;0;0;0;0];
q0arm = rad2deg([-1.283; -1.622; 2.2761853; -1.8021853; 1.112; 1.859; 0.05328]);
q0    = [q0b; deg2rad(q0arm)];

Tf = 210;  T_RL = 1;  Tlog = 0.1;  Tq = 0.1;  use_RL = 0;  sim_state = 1;

mdl = "concept3_simulink_RL";
open_system(mdl);

%% ══════════════════════════════════════════════════════
%  1단계: 참조 궤적 수집 (α = α_B 로 한 번 시뮬)
% ══════════════════════════════════════════════════════
fprintf('참조 궤적 수집 중 (null_method=alpha_B)...\n');
in = Simulink.SimulationInput(mdl);
in = setVariable(in, 'q0', q0, 'Workspace', mdl);
in = in.setBlockParameter(QP_BLK, 'null_method', 'alpha_B');
out_ref = sim(in);
fprintf('참조 시뮬 완료\n');

%% ══════════════════════════════════════════════════════
%  2단계: 로그 파싱
% ══════════════════════════════════════════════════════
state_raw = squeeze(out_ref.state.Data);
if size(state_raw, 1) ~= 27; state_raw = state_raw'; end

tt  = out_ref.state.Time;
N   = length(tt);
Ts  = mean(diff(tt));

q14_traj = [state_raw(1:7, :); state_raw(14:20, :)];   % 14×N

xdot_raw = squeeze(out_ref.xdot_des.Data);
tt_xdot  = out_ref.xdot_des.Time;
if size(xdot_raw, 1) ~= 6; xdot_raw = xdot_raw'; end
xdot_des_traj = interp1(tt_xdot, xdot_raw', tt, 'linear', 'extrap')';

%% ══════════════════════════════════════════════════════
%  3단계: bp, bv 전체 궤적 추출
% ══════════════════════════════════════════════════════
fprintf('궤적 추출 중 (N=%d)...\n', N);
[bp_traj, bv_traj, qdotp_traj, vnull_traj] = ...
    extract_traj(robot, q14_traj, xdot_des_traj, N);
fprintf('궤적 추출 완료\n');

%% ══════════════════════════════════════════════════════
%  4단계: Receding Horizon QP
% ══════════════════════════════════════════════════════
fprintf('Receding Horizon QP 시작 (H=%d 스텝 = %.1fs)...\n', H_hor, H_hor*Ts);

alpha_star = zeros(N, 1);
qdot_lim   = deg2rad(10) * ones(7, 1);
opts = optimoptions('quadprog', 'Display', 'off', ...
                    'Algorithm', 'interior-point-convex', ...
                    'MaxIterations', 500);

for k = 1:N
    k_end = min(k + H_hor - 1, N);
    H_win = k_end - k + 1;

    bp_w = bp_traj(:, k:k_end);      % 6×H_win
    bv_w = bv_traj(:, k:k_end);      % 6×H_win
    vn_w = vnull_traj(:, k:k_end);   % 7×H_win
    qp_w = qdotp_traj(:, k:k_end);   % 7×H_win

    % Hessian: 반동 비용 + 스무딩 (희소 삼대각)
    d_main = sum(bv_w .^ 2, 1)' + 2*lambda_s;
    d_off  = -lambda_s * ones(H_win-1, 1);
    H_qp = spdiags(d_main,    0, H_win, H_win) + ...
           spdiags([d_off;0], -1, H_win, H_win) + ...
           spdiags([d_off;0], +1, H_win, H_win);
    H_qp = (H_qp + H_qp') / 2;

    f_qp = 2 * sum(bp_w .* bv_w, 1)';

    % 관절 속도 제약 → α 구간
    lb_a = -inf(H_win, 1);
    ub_a =  inf(H_win, 1);
    for i = 1:H_win
        for j = 1:7
            vn_j = vn_w(j, i);
            qp_j = qp_w(j, i);
            if abs(vn_j) < 1e-8; continue; end
            hi = ( qdot_lim(j) - qp_j) / vn_j;
            lo = (-qdot_lim(j) - qp_j) / vn_j;
            if vn_j < 0; [hi, lo] = deal(lo, hi); end
            lb_a(i) = max(lb_a(i), lo);
            ub_a(i) = min(ub_a(i), hi);
        end
        if lb_a(i) > ub_a(i)
            mid = (lb_a(i) + ub_a(i)) / 2;
            lb_a(i) = mid - deg2rad(5);
            ub_a(i) = mid + deg2rad(5);
        end
    end

    [alpha_w, ~, exitflag] = quadprog(H_qp, f_qp, [], [], [], [], lb_a, ub_a, [], opts);

    if exitflag <= 0
        % QP 실패 시 greedy α_B 폴백
        Bv = bv_w(:, 1);  Bp = bp_w(:, 1);
        denom = Bv' * Bv;
        alpha_w = zeros(H_win, 1);
        if denom > 1e-10
            alpha_w(1) = -(Bp' * Bv) / denom;
        end
    end

    alpha_star(k) = alpha_w(1);   % receding: 첫 번째 값만 채택

    if mod(k, 300) == 0
        fprintf('  %d / %d\n', k, N);
    end
end

fprintf('Receding Horizon QP 완료\n');

%% ══════════════════════════════════════════════════════
%  5단계: 저장
% ══════════════════════════════════════════════════════
alpha_star_ts = timeseries(alpha_star, tt);
alpha_star_ts.Name = 'alpha_star';
save('alpha_star_offline.mat', 'alpha_star', 'alpha_star_ts', 'tt', 'lambda_s');
fprintf('alpha_star_offline.mat 저장 완료\n');

%% ══════════════════════════════════════════════════════
%  결과 시각화
% ══════════════════════════════════════════════════════
alpha_B_ref = -sum(bv_traj .* bp_traj, 1)' ./ max(sum(bv_traj.^2, 1)', 1e-10);

figure('Name', sprintf('RH QP (H=%d)', H_hor));

subplot(2,1,1);
plot(tt, alpha_B_ref, 'k--', 'LineWidth', 1.0, 'DisplayName', 'greedy α_B'); hold on;
plot(tt, alpha_star,  'b-',  'LineWidth', 1.4, 'DisplayName', sprintf('RH QP H=%d', H_hor));
legend('Location', 'best'); ylabel('α'); title('α*(t) 비교'); grid on;

subplot(2,1,2);
bv_sq  = sum(bv_traj.^2, 1)';
bp_bv  = sum(bp_traj .* bv_traj, 1)';
cost_B  = sum(bv_sq .* alpha_B_ref.^2 + 2*bp_bv .* alpha_B_ref) * Ts;
cost_RH = sum(bv_sq .* alpha_star.^2  + 2*bp_bv .* alpha_star)  * Ts;
bar([cost_B, cost_RH]);
set(gca, 'XTickLabel', {'greedy α_B', sprintf('RH H=%d', H_hor)});
ylabel('누적 반동 비용'); title('비용 비교 (참조 궤적 기준)'); grid on;

fprintf('\n비용: greedy=%.4f  RH=%.4f  개선=%.1f%%\n', ...
        cost_B, cost_RH, (cost_B - cost_RH) / abs(cost_B) * 100);

%% ══════════════════════════════════════════════════════
%  로컬 함수
% ══════════════════════════════════════════════════════
function [bp, bv, qdotp, vnull] = extract_traj(robot, q14_traj, xdot_des_traj, N)
bp    = zeros(6, N);
bv    = zeros(6, N);
qdotp = zeros(7, N);
vnull = zeros(7, N);

sigma_th   = 0.05;
lambda_max = 0.1;

for k = 1:N
    q14_k  = q14_traj(:, k);
    xdot_k = xdot_des_traj(:, k);

    J_full = geometricJacobian(robot, q14_k, 'ee');
    Jg     = J_full(:, 7:13);

    [U_k, S_mat, V] = svd(Jg);
    sv  = diag(S_mat);
    nsv = length(sv);

    m_manip = prod(sv);
    w_rot   = min(1.0, m_manip / 2.0);
    xdot_k(1:3) = w_rot * xdot_k(1:3);

    lambda_i = zeros(nsv, 1);
    for i = 1:nsv
        if sv(i) < sigma_th
            lambda_i(i) = lambda_max * (1 - (sv(i)/sigma_th)^2);
        end
    end
    sv_inv  = sv ./ (sv.^2 + lambda_i.^2);
    Jg_pinv = V(:,1:nsv) * diag(sv_inv) * U_k';

    qdot_p_k = Jg_pinv * xdot_k;
    v_null_k = V(:, end);

    H_k  = massMatrix(robot, q14_k);
    B_k  = H_k(1:6,1:6) \ H_k(1:6,7:13);

    bp(:,k)    = B_k * qdot_p_k;
    bv(:,k)    = B_k * v_null_k;
    qdotp(:,k) = qdot_p_k;
    vnull(:,k) = v_null_k;

    if mod(k, 300) == 0
        fprintf('    %d/%d\n', k, N);
    end
end
end

function port = AB2port(n)
    port_pos_data = [
     1.6 -0.35 0 -90 0 -90;
     1.1 -0.8  0   0 0  90;
     0.6 -0.8  0   0 0  90;
     0.1 -0.8  0   0 0  90;
    -0.4 -0.8  0   0 0  90;
     1.6  0.35 0 -90 0 -90;
     1.1  0.8  0   0 0 -90;
     0.6  0.8  0   0 0 -90;
     0.1  0.8  0   0 0 -90;
    -0.4  0.8  0   0 0 -90;
    ];
    port = port_pos_data(n,:);
end

function p = port_pos()
    p = [
     1.6 -0.35 0 -90 0 -90;
     1.1 -0.8  0   0 0  90;
     0.6 -0.8  0   0 0  90;
     0.1 -0.8  0   0 0  90;
    -0.4 -0.8  0   0 0  90;
     1.6  0.35 0 -90 0 -90;
     1.1  0.8  0   0 0 -90;
     0.6  0.8  0   0 0 -90;
     0.1  0.8  0   0 0 -90;
    -0.4  0.8  0   0 0 -90;
    ];
end

function [opacity, solid_mass] = set_obs(port_vector)
    mass       = 40;
    opacity    = port_vector;
    solid_mass = port_vector * mass;
end
