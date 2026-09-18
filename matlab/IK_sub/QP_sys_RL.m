classdef QP_sys_RL < matlab.System
% QP 기반 역기구학 + null-space RL 액션 — MATLAB System Object
%
% 입력 포트 (3개):
%   xdot_des  (6×1) : EE 목표 속도
%   q14       (14×1): [quat(4); pos(3); joint_angle(7)]
%   alpha     (1×1) : RL 액션 — null-space residual (Δα)
%
% 출력 포트 (3개):
%   qdot      (7×1) : 관절 속도 명령
%   alpha     (1×1) : 실제 사용된 null-space 파라미터 α (q2 안전 스케일 포함)
%   elapsed   (1×1) : 스텝 계산 시간 [s]
%
% 동작 구조:
%   1. DLS 슈도인버스 → qdot_p, v_null, m_manip
%   2. 질량행렬 → alpha_A(에너지 최소), alpha_B(베이스 반동 최소) 해석
%   3. qdot_cand = qdot_p + [0.5*(alpha_A+alpha_B) + Δα] * v_null
%   4. QP: min ||qdot − qdot_cand||²  s.t. 관절속도 + q2 위치 제약
%   5. QP 실패 시: qdot_cand 속도 클램핑 폴백
%
% ※ Simulink 블록 파라미터 → "다음을 사용하여 시뮬레이션" → 인터프리터형 실행

    properties (Access = private)
        robot
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
            obj.robot = loaded.robot;
            obj.robot.DataFormat = 'column';
        end

        function [qdot, alpha_out, elapsed] = stepImpl(obj, xdot_des, q14, alpha)
            t0 = tic;
            [qdot, alpha_out] = qp_rl_step(obj.robot, xdot_des, q14, alpha);
            elapsed = toc(t0);
        end

        function resetImpl(obj)
            loaded = load('concept3_for_sim.mat');
            obj.robot = loaded.robot;
            obj.robot.DataFormat = 'column';
        end

    end
end

% ===== 로컬 함수 =====
function [qdot, alpha_out] = qp_rl_step(robot, xdot_des, q14, delta_alpha)

%% 자코비안 및 SVD
J_full = geometricJacobian(robot, q14, 'ee');
Jg     = J_full(:, 7:13);   % 6×7 arm 자코비안

[U, S_mat, V] = svd(Jg);
sv   = diag(S_mat);          % 6×1 특이값
nsv  = length(sv);

%m_manip = prod(sv);          % Yoshikawa 조작성 지표

% 조작성 낮을 때 자세제어 약화 (특이점 근방: 병진 추종 우선)
%w_rot = min(1.0, m_manip / 2.0);
%xdot_des(1:3) = w_rot * xdot_des(1:3);

%% DLS 슈도인버스 (특이점 근방 안정화)
sigma_th   = 0.05;
lambda_max = 0.1;
lambda_i   = zeros(nsv, 1);
for i = 1:nsv
    if sv(i) < sigma_th
        lambda_i(i) = lambda_max * (1 - (sv(i)/sigma_th)^2);
    end
end
sv_inv  = sv ./ (sv.^2 + lambda_i.^2);
Jg_pinv = V(:,1:nsv) * diag(sv_inv) * U';   % 7×6

qdot_p = Jg_pinv * xdot_des;   % 7×1 particular solution
v_null = V(:, end);             % 7×1 null-space basis (7DOF − 6task = 1D)

%% alpha rate limiter — RL 액션 급변으로 인한 EE 이탈 방지
% T_RL=2s 기준 최대 허용 변화율: 0.3/step (alpha range [-1,1])
%persistent alpha_prev_rl
%if isempty(alpha_prev_rl); alpha_prev_rl = 0; end
%alpha_rate_lim = 0.3;
%delta_alpha = alpha_prev_rl + max(-alpha_rate_lim, min(alpha_rate_lim, delta_alpha - alpha_prev_rl));
%alpha_prev_rl = delta_alpha;

%% q2 안전 제어 — alpha 단 smooth 감쇠 (chattering 방지)
% 한계 쪽으로 미는 alpha 성분만 quadratic으로 0까지 줄임
% push-away 없음: 방향 전환이 chattering의 원인
q2_safe    = q14(9);       % [quat(4); pos(3); q1..q7] → q2 = 9번째
vn2        = v_null(2);
q2_min_lim = deg2rad(-135);
q2_max_lim = deg2rad(-30);
m_warn     = deg2rad(20);  % 감쇠 시작 거리 (넓게 잡아야 smooth 효과)


%delta_alpha = 1;

dist_max = q2_max_lim - q2_safe;   % 상한까지 남은 거리 (양수)
if dist_max < m_warn && delta_alpha * vn2 > 0
    scale = (max(0, dist_max) / m_warn)^2;   % quadratic: 1→0, 부드럽게 소멸
    delta_alpha = delta_alpha * scale;
end

dist_min = q2_safe - q2_min_lim;   % 하한까지 남은 거리 (양수)
if dist_min < m_warn && delta_alpha * vn2 < 0
    scale = (max(0, dist_min) / m_warn)^2;
    delta_alpha = delta_alpha * scale;
end

%% 후보 솔루션 구성
alpha_out   = 0.1 * delta_alpha;   % q2 안전 스케일 반영된 최종 α
qdot_cand   = qdot_p + alpha_out * v_null;

%qdot_cand   = qdot_p + 0.1* v_null;% + 0.2*delta_alpha* v_null;

%% QP: min 0.5‖qdot − qdot_cand‖²  s.t. 관절속도/위치 제약
qdot_lim = deg2rad([10; 10; 10; 10; 10; 10; 10]);  % [rad/s]

% q2 위치 제약 (마진 기반 soft limit)
q_arm  = q14(8:14);
q2     = q_arm(2);
q2_min = deg2rad(-135);
q2_max = deg2rad( -30);

% 마진 기반 soft limit: 한계 근처에서 속도를 비례적으로 줄이고,
% 한계 도달 시 추가 위반만 차단 (복귀 강제 없음 → 통통 튐 방지)
margin = deg2rad(5);   % 5° 연착 구간

lb =  -qdot_lim;
ub =   qdot_lim;

% 상한(q2_max) 처리
if q2 >= q2_max                          % 한계 초과: 양방향 이동 차단
    ub(2) = 0;
elseif q2 > q2_max - margin              % 마진 내: 비례 감속
    ub(2) = qdot_lim(2) * (q2_max - q2) / margin;
end

% 하한(q2_min) 처리
if q2 <= q2_min                          % 한계 초과: 음방향 이동 차단
    lb(2) = 0;
elseif q2 < q2_min + margin              % 마진 내: 비례 감속
    lb(2) = -qdot_lim(2) * (q2 - q2_min) / margin;
end

opts = optimoptions('quadprog', 'Display', 'off', ...
                    'Algorithm', 'interior-point-convex');
[qdot, ~, exitflag] = quadprog(eye(7), -qdot_cand, ...
                               [], [], [], [], lb, ub, [], opts);

if exitflag <= 0
    % QP 실패 시: lb/ub 클램핑 (위치 제약 포함)
    qdot = max(lb, min(ub, qdot_cand));
end

end
