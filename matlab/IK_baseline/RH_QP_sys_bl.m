classdef RH_QP_sys_bl < matlab.System
% Receding Horizon QP 역기구학 — MATLAB System Object
%
% 입력 포트 (3개):
%   xdot_des (6×1) : EE 목표 속도
%   q14      (14×1): [quat(4); pos(3); joint_angle(7)]
%   docked   (1×1) : 0=포워드(반동최소화) | 1=홈잉(베이스 자세 복귀)
%
% 출력 포트 (3개):
%   qdot     (7×1) : 관절 속도 명령
%   alpha    (1×1) : 실제 사용된 null-space 파라미터 α
%   elapsed  (1×1) : 스텝 계산 시간 [s]
%
% 모드:
%   docked=0 : min ||B·(qdot_p + α·v_null)||²  (전체 반동 최소화)
%   docked=1 : min ||B_ang·(qdot_p + α·v_null) - kp·e_att||²  (베이스 자세 복귀)
%              e_att = sign(qw)·[qx;qy;qz]

    properties
        H_hor    (1,1) double = 20    % 예측 구간 (타임스텝)
        lambda_s (1,1) double = 0.05  % 창 내 스무딩 강도
        lambda_t (1,1) double = 0.1   % 이전 α와 시간 연속성 강도
        kp_base  (1,1) double = 1.0   % 베이스 자세 복귀 게인 (docked=1 전용)
    end

    properties (Access = private)
        robot
        alpha_prev
    end

    methods (Access = protected)

        function num = getNumInputsImpl(~);  num = 3; end
        function num = getNumOutputsImpl(~); num = 3; end

        function [sz1, sz2, sz3] = getInputSizeImpl(~)
            sz1 = [6,  1];
            sz2 = [14, 1];
            sz3 = [1,  1];
        end

        function [sz1, sz2, sz3] = getOutputSizeImpl(~)
            sz1 = [7, 1];
            sz2 = [1, 1];
            sz3 = [1, 1];
        end

        function [dt1, dt2, dt3] = getInputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double';
        end

        function [dt1, dt2, dt3] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double';
        end

        function [c1, c2, c3] = isInputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end

        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false; c2 = false; c3 = false;
        end

        function [f1, f2, f3] = isInputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end

        function [f1, f2, f3] = isOutputFixedSizeImpl(~)
            f1 = true; f2 = true; f3 = true;
        end

        function setupImpl(obj)
            loaded = load('concept3_for_sim.mat');
            obj.robot      = loaded.robot;
            obj.robot.DataFormat = 'column';
            obj.alpha_prev = 0;
        end

        function [qdot, alpha, elapsed] = stepImpl(obj, xdot_des, q14, docked)
            t0 = tic;
            [qdot, alpha] = rh_qp_step(obj.robot, xdot_des, q14, docked, ...
                                        obj.H_hor, obj.lambda_s, ...
                                        obj.lambda_t, obj.kp_base, ...
                                        obj.alpha_prev);
            elapsed = toc(t0);
            obj.alpha_prev = alpha;
        end

        function resetImpl(obj)
            loaded = load('concept3_for_sim.mat');
            obj.robot      = loaded.robot;
            obj.robot.DataFormat = 'column';
            obj.alpha_prev = 0;
        end

    end
end

%% ===== 로컬 함수 =====
function [qdot, alpha_out] = rh_qp_step(robot, xdot_des, q14, docked, ...
                                         H_hor, lambda_s, lambda_t, ...
                                         kp_base, alpha_prev)

%% 자코비안 및 SVD
J_full = geometricJacobian(robot, q14, 'ee');
Jg     = J_full(:, 7:13);

[U, S_mat, V] = svd(Jg);
sv  = diag(S_mat);
nsv = length(sv);

m_manip = prod(sv);   %#ok<NASGU> 내부 w_rot 계산용
w_rot   = min(1.0, m_manip / 2.0);
xdot_des(1:3) = w_rot * xdot_des(1:3);

%% DLS 슈도인버스
sigma_th   = 0.05;
lambda_max = 0.1;
lambda_i   = zeros(nsv, 1);
for i = 1:nsv
    if sv(i) < sigma_th
        lambda_i(i) = lambda_max * (1 - (sv(i)/sigma_th)^2);
    end
end
sv_inv  = sv ./ (sv.^2 + lambda_i.^2);
Jg_pinv = V(:,1:nsv) * diag(sv_inv) * U';

qdot_p = Jg_pinv * xdot_des;
v_null = V(:, end);

%% B 행렬
H_mass = massMatrix(robot, q14);
B      = H_mass(1:6,1:6) \ H_mass(1:6,7:13);   % 6×7

%% 모드별 QP 비용 설정
if docked < 0.5
    %% ── 포워드: 전체 반동 최소화 ──────────────────────
    % min ||B·(qdot_p + α·v_null)||²
    bp_w = B * qdot_p;   % 6×1
    bv_w = B * v_null;   % 6×1
    bv_sq = bv_w' * bv_w;
    f_scalar = 2 * (bp_w' * bv_w);

else
    %% ── 홈잉: 베이스 각속도 → kp·e_att 추종 ──────────
    % 운동량 보존: omega_b = -B_ang·qdot
    % 원하는 omega_b = -kp·e_att (자세 오차 감쇠 방향)
    % → B_ang·(qdot_p + α·v_null) = kp·e_att
    % min ||B_ang·(qdot_p + α·v_null) - kp·e_att||²
    B_ang  = B(1:3, :);                        % 3×7
    e_att  = sign(q14(1)) * q14(2:4);          % [qx;qy;qz] 자세 오차
    omega_des = kp_base * e_att;               % 3×1 목표 각속도

    bp_w  = B_ang * qdot_p - omega_des;        % 3×1 편차 (상수항 포함)
    bv_w  = B_ang * v_null;                    % 3×1
    bv_sq = bv_w' * bv_w;
    f_scalar = 2 * (bp_w' * bv_w);
end

%% Receding Horizon QP
% Hessian: 삼대각 스무딩 + 시간 연속성
d_main    = bv_sq * ones(H_hor, 1) + 2*lambda_s;
d_off     = -lambda_s * ones(H_hor-1, 1);
H_qp      = spdiags(d_main,    0, H_hor, H_hor) + ...
             spdiags([d_off;0], -1, H_hor, H_hor) + ...
             spdiags([d_off;0], +1, H_hor, H_hor);
H_qp(1,1) = H_qp(1,1) + 2*lambda_t;   % 이전 α와 시간 연속성
H_qp      = (H_qp + H_qp') / 2;

f_qp    = f_scalar * ones(H_hor, 1);
f_qp(1) = f_qp(1) - 2*lambda_t * alpha_prev;

%% 관절 속도 제약 → α 구간 (현재 상태 기준)
qdot_lim = deg2rad([10;10;10;10;10;10;10]);
lb_a = -0.1 * ones(H_hor, 1);
ub_a =  0.1 * ones(H_hor, 1);
for j = 1:7
    vn_j = v_null(j);
    qp_j = qdot_p(j);
    if abs(vn_j) < 1e-8; continue; end
    hi = ( qdot_lim(j) - qp_j) / vn_j;
    lo = (-qdot_lim(j) - qp_j) / vn_j;
    if vn_j < 0; [hi, lo] = deal(lo, hi); end
    lb_a(:) = max(lb_a, lo);
    ub_a(:) = min(ub_a, hi);
end
for i = 1:H_hor
    if lb_a(i) > ub_a(i)
        mid = (lb_a(i) + ub_a(i)) / 2;
        lb_a(i) = mid - deg2rad(5);
        ub_a(i) = mid + deg2rad(5);
    end
end

opts = optimoptions('quadprog', 'Display', 'off', ...
                    'Algorithm', 'interior-point-convex', ...
                    'MaxIterations', 500);
[alpha_w, ~, exitflag] = quadprog(H_qp, f_qp, [], [], [], [], lb_a, ub_a, [], opts);

if exitflag <= 0
    % 폴백: greedy α_B (α 범위 동일하게 클램프)
    if bv_sq > 1e-10
        alpha_out = max(-0.1, min(0.1, -f_scalar / (2 * bv_sq)));
    else
        alpha_out = 0;
    end
else
    alpha_out = alpha_w(1);
end

%% 관절 속도 명령 + 위치 제약 QP
qdot_cand = qdot_p + alpha_out * v_null;

q2     = q14(9);
q2_min = deg2rad(-135);
q2_max = deg2rad(-30);
margin = deg2rad(5);

lb = -qdot_lim;
ub =  qdot_lim;

if q2 >= q2_max
    ub(2) = 0;
elseif q2 > q2_max - margin
    ub(2) = qdot_lim(2) * (q2_max - q2) / margin;
end
if q2 <= q2_min
    lb(2) = 0;
elseif q2 < q2_min + margin
    lb(2) = -qdot_lim(2) * (q2 - q2_min) / margin;
end

opts2 = optimoptions('quadprog', 'Display', 'off', ...
                     'Algorithm', 'interior-point-convex');
[qdot, ~, ef2] = quadprog(eye(7), -qdot_cand, [], [], [], [], lb, ub, [], opts2);
if ef2 <= 0
    qdot = max(lb, min(ub, qdot_cand));
end

end
