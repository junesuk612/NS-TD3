classdef QP_sys_bl < matlab.System
% QP 기반 역기구학 + null-space 베이스 제어 — MATLAB System Object
%
% 입력 포트 (3개):
%   xdot_des  (6×1) : EE 목표 속도
%   q14       (14×1): [quat(4); pos(3); joint_angle(7)]
%   alpha_ext (1×1) : 외부 α 입력 (null_method='external' 전용, 그 외 무시)
%
% 출력 포트 (3개):
%   qdot      (7×1) : 관절 속도 명령
%   m_manip   (1×1) : 조작성 지표 = prod(sv(Jg))
%   elapsed   (1×1) : 스텝 계산 시간 [s]
%
% null_method 선택:
%   'alpha_B'   : 베이스 반동 최소화 (greedy, α_B 해석해)
%   'alpha_BC'  : 베이스 자세 피드백 — 자세 오차를 null-space에 투영
%   'external'  : alpha_ext 입력을 그대로 사용 (offline α* 주입용)

    properties
        % 'alpha_B' | 'alpha_BC' | 'external'
        null_method (1,:) char = 'alpha_B'
        % 베이스 자세 피드백 게인 (alpha_BC 전용)
        kp_base (1,1) double = 1.0
    end

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

        function [qdot, m_manip, elapsed] = stepImpl(obj, xdot_des, q14, alpha_ext)
            t0 = tic;
            [qdot, m_manip] = qp_bl_step(obj.robot, xdot_des, q14, ...
                                          obj.null_method, obj.kp_base, alpha_ext);
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
function [qdot, m_manip] = qp_bl_step(robot, xdot_des, q14, null_method, kp_base, alpha_ext)

%% 자코비안 및 SVD
J_full = geometricJacobian(robot, q14, 'ee');
Jg     = J_full(:, 7:13);   % 6×7 arm 자코비안

[U, S_mat, V] = svd(Jg);
sv   = diag(S_mat);          % 6×1 특이값
nsv  = length(sv);

m_manip = prod(sv);          % Yoshikawa 조작성 지표

% 조작성 낮을 때 자세제어 약화 (특이점 근방: 병진 추종 우선)
w_rot = min(1.0, m_manip / 2.0);
xdot_des(1:3) = w_rot * xdot_des(1:3);

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
%v_null = V(:, end);             % 7×1 null-space basis (7DOF − 6task = 1D)

%% 질량행렬 → B 행렬 (두 방법 공통)
% B = Hbb\Hba : 팔 운동이 베이스에 전달하는 관성 연성 (omega_b = -B * qdot)
%H   = massMatrix(robot, q14);   % 13×13 (base 6 + arm 7)
%Hbb = H(1:6, 1:6);
%Hba = H(1:6, 7:13);
%B   = Hbb \ Hba;                % 6×7

%% null-space alpha 선택
%{
switch null_method

    case 'alpha_B'
        % 베이스 반동 최소화: greedy α_B 해석해
        % min ||B*(qdot_p + α*v_null)||² → α_B = -(B*qdot_p)'*(B*v_null) / ||B*v_null||²
        Bv    = B * v_null;
        Bqp   = B * qdot_p;
        denom = Bv' * Bv;
        if denom > 1e-10
            alpha = -(Bqp' * Bv) / denom;
        else
            alpha = 0;
        end

    case 'alpha_BC'
        % 베이스 자세 피드백: 자세 오차 → 팔이 베이스를 돌리는 방향
        %
        % 운동량 보존: omega_b = -B_ang * qdot
        % 원하는 베이스 각속도: omega_cmd = -kp * e_att
        % B_ang * qdot = kp * e_att → qdot_target = pinv(B_ang) * kp * e_att
        % null-space 투영: alpha_BC = v_null' * qdot_target
        %
        % q14(1) = qw, q14(2:4) = [qx;qy;qz]
        e_att = sign(q14(1)) * q14(2:4);   % 자세 오차 벡터 (최단 경로 보장)
        B_ang = B(1:3, :);                  % 각속도 연성만 (3×7)

        % pinv(B_ang) = B_ang' * inv(B_ang * B_ang')  (full row rank 가정)
        BBt = B_ang * B_ang';
        qdot_target = B_ang' * (BBt \ (kp_base * e_att));   % 7×1
        alpha = v_null' * qdot_target;

    case 'external'
        % 외부 α 직접 사용 (alpha_star_lookup 블록 출력 주입용)
        alpha = alpha_ext;

    otherwise
        alpha = 0;
end
%}
%% 후보 솔루션 구성
qdot_cand = qdot_p;% + alpha * v_null;
qdot = qdot_cand;   % QP 솔버 실패 시 fallback (제한조건 무시)

%{
%% QP: min 0.5‖qdot − qdot_cand‖²  s.t. 관절속도/위치 제약
qdot_lim = deg2rad([10; 10; 10; 10; 10; 10; 10]);  % [rad/s]

% q2 위치 제약 (마진 기반 soft limit)
q_arm  = q14(8:14);
q2     = q_arm(2);
q2_min = deg2rad(-135);
q2_max = deg2rad( -30);

margin = deg2rad(5);   % 5° 연착 구간

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

opts = optimoptions('quadprog', 'Display', 'off', ...
                    'Algorithm', 'interior-point-convex');
[qdot, ~, exitflag] = quadprog(eye(7), -qdot_cand, ...
                               [], [], [], [], lb, ub, [], opts);

if exitflag <= 0
    % QP 실패 시: lb/ub 클램핑 (위치 제약 포함)
    qdot = max(lb, min(ub, qdot_cand));
end
%}
end
