classdef Pseudo_inv_sys_RL < matlab.System
% DLS 슈도인버스 역기구학 + null-space RL 액션 — MATLAB System Object
% QP_sys_RL 동등 구현: quadprog 제거, box clamp로 대체
%
% 입력 포트 (3개):
%   xdot_des    (6×1) : EE 목표 속도
%   q14         (14×1): [quat(4); pos(3); joint_angle(7)]
%   delta_alpha (1×1) : RL 액션 — null-space residual (Δα)
%
% 출력 포트 (3개):
%   qdot      (7×1) : 관절 속도 명령
%   alpha_out (1×1) : 실제 사용된 null-space 파라미터 α (q2 안전 스케일 포함)
%   elapsed   (1×1) : 스텝 계산 시간 [s]
%
% 동작 구조:
%   1. DLS 슈도인버스 → qdot_p, v_null
%   2. q2 smooth alpha 감쇠 (한계 근방 chattering 방지)
%   3. qdot_cand = qdot_p + alpha_out * v_null
%   4. 관절속도/q2 위치 제약 box clamp (quadprog 등가)
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

        function [qdot, alpha_out, elapsed] = stepImpl(obj, xdot_des, q14, delta_alpha)
            t0 = tic;
            [qdot, alpha_out] = pinv_rl_step(obj.robot, xdot_des, q14, delta_alpha);
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
function [qdot, alpha_out] = pinv_rl_step(robot, xdot_des, q14, delta_alpha)

%% 자코비안 및 SVD
J_full = geometricJacobian(robot, q14, 'ee');
Jg     = J_full(:, 7:13);   % 6×7 arm 자코비안

[U, S_mat, V] = svd(Jg);
sv   = diag(S_mat);          % 6×1 특이값
nsv  = length(sv);

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

%% 상수 (deg2rad 반복 호출 제거 — 계산 시간 절감)
Q2_MIN_LIM = -2.35619449019234;   % deg2rad(-135)
Q2_MAX_LIM = -0.52359877559830;   % deg2rad(-30)
M_WARN     =  0.34906585039887;   % deg2rad(20)
QDOT_LIM   =  0.17453292519943;   % deg2rad(10)
Q2_MARGIN  =  0.08726646259972;   % deg2rad(5)

%% q2 안전 제어 — alpha smooth 감쇠 (chattering 방지)
% 한계 쪽으로 미는 alpha 성분만 quadratic으로 0까지 줄임
q2_safe = q14(9);
vn2     = v_null(2);

dist_max = Q2_MAX_LIM - q2_safe;
if dist_max < M_WARN && delta_alpha * vn2 > 0
    scale = (max(0, dist_max) / M_WARN)^2;
    delta_alpha = delta_alpha * scale;
end

dist_min = q2_safe - Q2_MIN_LIM;
if dist_min < M_WARN && delta_alpha * vn2 < 0
    scale = (max(0, dist_min) / M_WARN)^2;
    delta_alpha = delta_alpha * scale;
end

%% 후보 솔루션 구성
alpha_out = 0.1 * delta_alpha;
qdot_cand = qdot_p + alpha_out * v_null;

%% 관절속도 + q2 위치 제약 (box clamp — quadprog 등가)
qdot_lim = QDOT_LIM * ones(7,1);

q2     = q14(9);
q2_min = Q2_MIN_LIM;
q2_max = Q2_MAX_LIM;
margin = Q2_MARGIN;

lb = -qdot_lim;
ub =  qdot_lim;

% 상한(q2_max) 처리
if q2 >= q2_max
    ub(2) = 0;
elseif q2 > q2_max - margin
    ub(2) = qdot_lim(2) * (q2_max - q2) / margin;
end

% 하한(q2_min) 처리
if q2 <= q2_min
    lb(2) = 0;
elseif q2 < q2_min + margin
    lb(2) = -qdot_lim(2) * (q2 - q2_min) / margin;
end

qdot = max(lb, min(ub, qdot_cand));

end
