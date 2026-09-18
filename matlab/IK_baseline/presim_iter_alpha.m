%% presim_iter_alpha.m — 반복 정제로 오프라인 최적 α*(t) 계산
%
% 알고리즘:
%   iter 1 : α=α_B (greedy) 로 시뮬 → QP → α_1*(t) 저장
%   iter 2+: α_{k-1}*(t) 적용해 재시뮬 → 새 궤적 데이터 → QP → α_k*(t) 저장
%   → 2~3회면 수렴 (동결 궤적 오류 해소)
%
% Simulink 사전 준비:
%   [1] xdot_des 신호 To Workspace 로그 (presim_offline_alpha와 동일)
%   [2] QP_sys_bl 블록 경로를 아래 QP_BLK 변수에 입력
%       (Simulink 모델 열고, QP_sys_bl 블록 우클릭 → Properties → Full path 확인)

clc; clear; close all;
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

%% ══════════════════════════════════════════════════════
%  설정
% ══════════════════════════════════════════════════════
N_iter   = 100;      % 반복 횟수 (보통 2~3회면 수렴)
lambda_s = 0.05;   % 스무딩 강도
w_T      = 500;    % 종단 자세복귀 소프트 페널티 가중치
beta     = 0.5;    % 반복 감쇠 계수 (진동 방지, 0<β≤1)

% QP_sys_bl 블록 전체 경로 (Simulink 모델 내 경로 확인 후 수정)
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
%  반복 루프
% ══════════════════════════════════════════════════════
alpha_history = cell(N_iter, 1);
cost_history  = zeros(N_iter, 1);
conv_history  = zeros(N_iter, 1);
tt_ref        = [];
alpha_prev    = [];

for iter = 1:N_iter
    fprintf('\n══════ Iteration %d / %d ══════\n', iter, N_iter);

    %% ── 1. Simulink 실행 ──────────────────────────────
    in = Simulink.SimulationInput(mdl);
    in = setVariable(in, 'q0', q0, 'Workspace', mdl);

    if iter == 1
        % 첫 번째: greedy α_B 로 참조 궤적 수집
        in = in.setBlockParameter(QP_BLK, 'null_method', 'alpha');
        fprintf('  시뮬: null_method=alpha (참조 궤적)\n');
    else
        % 이후: 직전 α* 적용 (alpha_star_offline.mat 로부터 alpha_star_lookup이 읽음)
        in = in.setBlockParameter(QP_BLK, 'null_method', 'external');
        fprintf('  시뮬: null_method=external (직전 α* 적용)\n');
    end

    out_i = sim(in);
    fprintf('  시뮬 완료\n');

    %% ── 2. 로그 파싱 ──────────────────────────────────
    state_raw = squeeze(out_i.state.Data);
    if size(state_raw, 1) ~= 27; state_raw = state_raw'; end

    tt   = out_i.state.Time;
    N    = length(tt);
    Ts   = mean(diff(tt));

    % q14: [quat(4); pos(3); q_arm(7)]
    q14_traj = [state_raw(1:7, :); state_raw(14:20, :)];   % 14×N

    % xdot_des: 시간축 보간
    xdot_raw = squeeze(out_i.xdot_des.Data);
    tt_xdot  = out_i.xdot_des.Time;
    if size(xdot_raw, 1) ~= 6; xdot_raw = xdot_raw'; end
    xdot_des_traj = interp1(tt_xdot, xdot_raw', tt, 'linear', 'extrap')';

    %% ── 3. qdot_p / v_null / B 재계산 ────────────────
    fprintf('  궤적 추출 중 (N=%d)...\n', N);
    [bp_traj, bv_traj, qdotp_traj, vnull_traj] = ...
        extract_traj(robot, q14_traj, xdot_des_traj, N);
    fprintf('  궤적 추출 완료\n');

    %% ── 4. QP 조립 및 풀기 ────────────────────────────
    % Hessian (희소 삼대각)
    d_main = sum(bv_traj .^ 2, 1)' + 2*lambda_s;
    d_off  = -lambda_s * ones(N-1, 1);
    H_sp = spdiags(d_main,    0, N, N) + ...
           spdiags([d_off;0], -1, N, N) + ...
           spdiags([d_off;0], +1, N, N);
    H_sp = (H_sp + H_sp') / 2;

    f_qp = 2 * sum(bp_traj .* bv_traj, 1)';

    % 소프트 종단 제약: w_T·||A_eq·α - b_eq||² 비용에 추가
    % 하드 등호 대신 소프트로 → b_eq 변화에도 해가 연속적으로 변해 진동 방지
    bp_ang = bp_traj(1:3, :);
    bv_ang = bv_traj(1:3, :);
    q_vec0 = q14_traj(2:4, 1);
    A_eq    = (Ts/2) * bv_ang;                       % 3×N
    b_eq    = q_vec0 + (Ts/2) * sum(bp_ang, 2);      % 3×1
    % quadprog: min 0.5·α'Hα + f'α  +  w_T·||A·α-b||²
    % 전개: H += 2·w_T·A'A,  f += -2·w_T·A'b
    A_eq_sp = sparse(A_eq);
    H_soft  = H_sp + 2*w_T * (A_eq_sp' * A_eq_sp);
    H_soft  = (H_soft + H_soft') / 2;
    f_soft  = f_qp - 2*w_T * (A_eq' * b_eq);

    % 관절 속도 제약 → α 구간
    qdot_lim = deg2rad(10) * ones(7, 1);
    lb_a = -inf(N, 1);
    ub_a =  inf(N, 1);
    for k = 1:N
        for j = 1:7
            vn_j = vnull_traj(j, k);
            qp_j = qdotp_traj(j, k);
            if abs(vn_j) < 1e-8; continue; end
            hi = ( qdot_lim(j) - qp_j) / vn_j;
            lo = (-qdot_lim(j) - qp_j) / vn_j;
            if vn_j < 0; [hi, lo] = deal(lo, hi); end
            lb_a(k) = max(lb_a(k), lo);
            ub_a(k) = min(ub_a(k), hi);
        end
        if lb_a(k) > ub_a(k)
            mid = (lb_a(k) + ub_a(k)) / 2;
            lb_a(k) = mid - deg2rad(5);
            ub_a(k) = mid + deg2rad(5);
        end
    end

    opts = optimoptions('quadprog', 'Display', 'off', ...
                        'Algorithm', 'interior-point-convex', ...
                        'MaxIterations', 1000);
    [alpha_qp, cost_val, exitflag] = quadprog(H_soft, f_soft, ...
                                     [], [], [], [], lb_a, ub_a, [], opts);
    if exitflag <= 0
        warning('  iter %d: QP 수렴 미달 (exitflag=%d)', iter, exitflag);
    end
    cost_history(iter) = cost_val;
    fprintf('  QP 비용 = %.6f  종단오차 = %.4f\n', cost_val, norm(A_eq*alpha_qp - b_eq));

    % 감쇠 업데이트: α_new = (1-β)·α_prev + β·α_qp (진동 억제)
    if iter == 1 || isempty(alpha_prev)
        alpha_new = alpha_qp;
    else
        alpha_prev_i = interp1(tt_ref, alpha_prev, tt, 'linear', 'extrap');
        alpha_new    = (1 - beta) * alpha_prev_i + beta * alpha_qp;
    end

    %% ── 5. 수렴 확인 ──────────────────────────────────
    if iter > 1
        conv_err = norm(alpha_new - interp1(tt_ref, alpha_prev, tt, 'linear', 'extrap')) ...
                   / (norm(alpha_prev) + 1e-10);
        conv_history(iter) = conv_err;
        fprintf('  수렴 오차 ||Δα||/||α|| = %.4f\n', conv_err);
    end

    alpha_history{iter} = alpha_new;
    alpha_prev = alpha_new;
    tt_ref     = tt;

    %% ── 6. α* 저장 (다음 iter에서 alpha_star_lookup이 읽음) ──
    alpha_star    = alpha_new;
    alpha_star_ts = timeseries(alpha_star, tt);
    alpha_star_ts.Name = 'alpha_star';
    save('alpha_star_offline.mat', 'alpha_star', 'alpha_star_ts', 'tt', 'lambda_s');
    fprintf('  alpha_star_offline.mat 갱신 완료\n');
end

%% ══════════════════════════════════════════════════════
%  결과 시각화 — 반복별 α* 비교
% ══════════════════════════════════════════════════════
alpha_B_ref = -sum(bv_traj .* bp_traj, 1)' ./ max(sum(bv_traj.^2, 1)', 1e-10);

figure('Name', '반복 정제 결과');

subplot(3,1,1);
plot(tt, alpha_B_ref, 'k--', 'LineWidth', 1.0, 'DisplayName', 'greedy α_B'); hold on;
colors = lines(N_iter);
for i = 1:N_iter
    if ~isempty(alpha_history{i})
        plot(tt, alpha_history{i}, 'LineWidth', 1.4, ...
             'Color', colors(i,:), 'DisplayName', sprintf('iter %d', i));
    end
end
legend('Location', 'best'); ylabel('α'); title('반복별 α*(t)'); grid on;

subplot(3,1,2);
valid = find(conv_history > 0);
if ~isempty(valid)
    plot(valid, conv_history(valid), 'ro-', 'LineWidth', 1.4);
    xlabel('iteration'); ylabel('||Δα||/||α||'); title('수렴 오차'); grid on;
end

subplot(3,1,3);
bar(1:N_iter, cost_history);
xlabel('iteration'); ylabel('QP 비용'); title('반복별 최적화 비용'); grid on;

fprintf('\n최종 α* → alpha_star_offline.mat 에 저장됨\n');

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

    m_manip   = prod(sv);
    w_rot     = min(1.0, m_manip / 2.0);
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
