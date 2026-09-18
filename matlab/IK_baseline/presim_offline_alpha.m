%% presim_offline_alpha.m — 오프라인 최적 α*(t) 계산
%
% ═══════════════════════════════════════════════════════════════
%  Simulink 사전 준비 (최초 1회만):
%
%  [1] xdot_des (6×1) 신호 To Workspace 추가
%      - 위치: 궤적 생성기 → QP 블록 사이의 xdot_des 신호선
%      - 블록: To Workspace
%        · 변수명: xdot_des
%        · SaveFormat: Array  (또는 Timeseries)
%        · SampleTime: -1  (신호 주기 그대로)
%
%  [참고] out.state (27D)는 이미 로그됨 — 추가 불필요
%  [참고] q14(t) = state([1:7, 14:20], :) 로 바로 추출 가능
%
%  Simulink에서 xdot_des 신호선 찾는 법:
%    Simulink 모델 열기 → 궤적 블록 출력선 우클릭
%    → Properties → Logging → Log signal 체크
%    → 변수명 'xdot_des', Format Array
% ═══════════════════════════════════════════════════════════════

clc; clear; close all;
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

%% ────────────────────────────────────────────
%  0. 공통 초기 조건 (baseline_sim과 동일)
% ────────────────────────────────────────────
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

% Simulink 모델이 기본 작업공간에서 참조하는 타이밍/제어 변수
Tf        = 210;
T_RL      = 1;
Tlog      = 0.1;
Tq        = 0.1;
use_RL    = 0;
sim_state = 1;

%% ────────────────────────────────────────────
%  1단계: 참조 시뮬레이션 실행 (α = α_B greedy)
%  목적: state(t), xdot_des(t) 수집
% ────────────────────────────────────────────
mdl = "concept3_simulink_RL";
open_system(mdl);

in = Simulink.SimulationInput(mdl);
in = setVariable(in, 'q0', q0, 'Workspace', mdl);
% QP_sys_bl.null_method = 'alpha_B' (그리디 — 참조 궤적 생성)
% Simulink 블록 마스크에서 null_method를 'alpha_B'로 설정 후 실행

out_ref = sim(in);
fprintf('[1단계] 참조 시뮬레이션 완료\n');

%% ────────────────────────────────────────────
%  2단계: 로그 파싱
%  out.state  (27D): [qw qx qy qz | x y z | vb_lin(3) vb_ang(3) | q_arm(7) | qdot_arm(7)]
%  out.xdot_des (6D): EE 속도 명령 (Simulink To Workspace 필요)
% ────────────────────────────────────────────
state_raw = squeeze(out_ref.state.Data);   % 27×N 또는 N×27
if size(state_raw, 1) ~= 27; state_raw = state_raw'; end

% state 시간축을 기준 그리드로 사용
tt = out_ref.state.Time;    % N×1
N  = length(tt);
Ts = mean(diff(tt));

% q14(t): [quat(4); pos(3); q_arm(7)]
%  state 인덱스: 1:4=quat, 5:7=pos, 8:10=vb_lin, 11:13=vb_ang, 14:20=q_arm
q14_traj = [state_raw(1:7, :); state_raw(14:20, :)];   % 14×N

% xdot_des(t): state와 샘플링 주기가 다를 수 있으므로 tt 기준으로 보간
xdot_raw  = squeeze(out_ref.xdot_des.Data);   % 6×M 또는 M×6
tt_xdot   = out_ref.xdot_des.Time;            % M×1 (솔버 스텝 주기)
if size(xdot_raw, 1) ~= 6; xdot_raw = xdot_raw'; end

% tt(state) 기준으로 선형 보간 → 6×N
xdot_des_traj = interp1(tt_xdot, xdot_raw', tt, 'linear', 'extrap')';

fprintf('[2단계] 데이터 파싱 완료: N=%d 스텝, Ts=%.3fs\n', N, Ts);
fprintf('        state: %d샘플(%.4fs), xdot_des: %d샘플 → 보간\n', ...
        N, Ts, length(tt_xdot));

%% ────────────────────────────────────────────
%  3단계: 각 타임스텝에서 qdot_p, v_null, B 재계산
%  (Simulink 내부와 동일한 연산 — 로봇 상태 이용)
% ────────────────────────────────────────────
fprintf('[3단계] 궤적 데이터 추출 중 (총 %d 스텝)...\n', N);

bp_traj    = zeros(6, N);   % B * qdot_p
bv_traj    = zeros(6, N);   % B * v_null
qdotp_traj = zeros(7, N);   % qdot_p
vnull_traj = zeros(7, N);   % v_null

sigma_th   = 0.05;
lambda_max = 0.1;

for k = 1:N
    q14_k   = q14_traj(:, k);
    xdot_k  = xdot_des_traj(:, k);

    % --- 자코비안 & DLS 슈도인버스 ---
    J_full  = geometricJacobian(robot, q14_k, 'ee');
    Jg      = J_full(:, 7:13);           % 6×7 arm Jacobian

    [U_k, S_mat, V] = svd(Jg);
    sv   = diag(S_mat);
    nsv  = length(sv);

    % 조작성 낮을 때 자세 명령 약화 (QP_sys_bl과 동일)
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
    Jg_pinv = V(:,1:nsv) * diag(sv_inv) * U_k';   % 7×6

    qdot_p_k = Jg_pinv * xdot_k;   % 7×1
    v_null_k = V(:, end);            % 7×1

    % --- 질량행렬 → B ---
    H_k  = massMatrix(robot, q14_k);   % 13×13
    Hbb  = H_k(1:6, 1:6);
    Hba  = H_k(1:6, 7:13);
    B_k  = Hbb \ Hba;                  % 6×7

    bp_traj(:, k)    = B_k * qdot_p_k;
    bv_traj(:, k)    = B_k * v_null_k;
    qdotp_traj(:, k) = qdot_p_k;
    vnull_traj(:, k) = v_null_k;

    if mod(k, 200) == 0
        fprintf('  %d / %d 완료\n', k, N);
    end
end
fprintf('[3단계] 완료\n');

%% ────────────────────────────────────────────
%  4단계: 오프라인 QP 조립 (종단 자세 등호 제약 포함)
%
%  목적함수:
%    min_α  Σ_t ||bp(:,t) + α(t)*bv(:,t)||²  +  λ_s*Σ_t (α(t+1)−α(t))²
%
%  등호 제약 — 종단 베이스 자세 복귀 (소각도 선형화):
%    ω_base(t) = -bp_ang(t) - α(t)*bv_ang(t)
%    q_vec(T)  = q_vec(0) + (Ts/2)*Σ_t ω_base(t)  →  =0 조건
%    ⟹  (Ts/2)*bv_ang_mat*α  =  q_vec(0) + (Ts/2)*Σ_t bp_ang(t)
%
%  부등호 제약:
%    관절 속도 한계: -qdot_lim ≤ qdot_p(:,t) + α(t)*vnull(:,t) ≤ qdot_lim
% ────────────────────────────────────────────
lambda_s = 0.05;   % 스무딩 강도

%% Hessian (희소 삼대각)
d_main = sum(bv_traj .^ 2, 1)' + 2*lambda_s;
d_off  = -lambda_s * ones(N-1, 1);

H_sp = spdiags(d_main, 0, N, N) + ...
       spdiags([d_off; 0], -1, N, N) + ...
       spdiags([d_off; 0], +1, N, N);
H_sp = (H_sp + H_sp') / 2;

%% 선형 항
f_qp = 2 * sum(bp_traj .* bv_traj, 1)';   % N×1

%% 등호 제약: 종단 베이스 자세 q_vec(T) = 0
% bp_ang, bv_ang: B 행렬의 각속도 연성 부분 (상위 3행)
bp_ang = bp_traj(1:3, :);   % 3×N
bv_ang = bv_traj(1:3, :);   % 3×N

q_vec0 = q14_traj(2:4, 1);   % 초기 베이스 [qx;qy;qz]

A_eq = (Ts/2) * bv_ang;                         % 3×N
b_eq = q_vec0 + (Ts/2) * sum(bp_ang, 2);        % 3×1

fprintf('[4단계] 종단 자세 제약 우변 크기: ||b_eq|| = %.4f rad\n', norm(b_eq));

%% 부등호 제약: 관절 속도 한계 → 타임스텝별 α 구간
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

%% quadprog
opts = optimoptions('quadprog', 'Display', 'iter', ...
                    'Algorithm', 'interior-point-convex', ...
                    'MaxIterations', 1000);

fprintf('[4단계] 오프라인 QP 풀기 (N=%d 변수, 등호제약 3개)...\n', N);
[alpha_star, cost_val, exitflag] = quadprog(H_sp, f_qp, ...
                                   [], [], A_eq, b_eq, lb_a, ub_a, [], opts);

if exitflag > 0
    fprintf('[4단계] 최적화 성공  비용 = %.6f\n', cost_val);
else
    warning('[4단계] quadprog 수렴 미달 (exitflag=%d)\n', exitflag);
end

%% ────────────────────────────────────────────
%  5단계: 결과 저장
%  → Simulink From Workspace 블록으로 주입 가능한 형식
% ────────────────────────────────────────────
% Simulink From Workspace는 [time, data] 행렬 또는 timeseries 입력
alpha_star_ts = timeseries(alpha_star, tt);
alpha_star_ts.Name = 'alpha_star';

save('alpha_star_offline.mat', 'alpha_star', 'alpha_star_ts', 'tt', 'lambda_s');
fprintf('[5단계] 저장 완료 → alpha_star_offline.mat\n');

%% ────────────────────────────────────────────
%  6단계: greedy α_B vs offline α* 시각화
% ────────────────────────────────────────────
alpha_B_ref = -sum(bp_traj .* bv_traj, 1)' ./ ...
               max(sum(bv_traj .^ 2, 1)', 1e-10);

figure('Name', '오프라인 α* vs greedy α_B');

subplot(3,1,1);
plot(tt, alpha_B_ref, 'b--', 'LineWidth', 1.2); hold on;
plot(tt, alpha_star, 'r-',  'LineWidth', 1.5);
legend('greedy α_B', sprintf('offline α* (λ_s=%.3f)', lambda_s));
ylabel('α'); title('null-space 파라미터'); grid on;

subplot(3,1,2);
plot(tt, alpha_star - alpha_B_ref, 'k-', 'LineWidth', 1.2);
ylabel('Δα'); title('offline α* − greedy α_B'); grid on;

subplot(3,1,3);
base_vel_norm_ref  = vecnorm(bp_traj + alpha_B_ref' .* bv_traj, 2, 1);
base_vel_norm_star = vecnorm(bp_traj + alpha_star' .* bv_traj, 2, 1);
plot(tt, base_vel_norm_ref,  'b--', 'LineWidth', 1.2); hold on;
plot(tt, base_vel_norm_star, 'r-',  'LineWidth', 1.5);
legend('greedy α_B', 'offline α*');
ylabel('||B·qdot||'); xlabel('t [s]');
title('베이스 유발 속도 노름 (작을수록 좋음)'); grid on;

%% ===== 로컬 함수 =====
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
    mass    = 40;
    opacity = port_vector;
    solid_mass = port_vector * mass;
end
