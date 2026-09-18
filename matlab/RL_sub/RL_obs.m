function obs = RL_obs(u)
%#codegen
% RL_obs  concept3 강화학습 관측 함수
%
% 입력 u (41×1):
%   u(1:3)   = EE 오차            [ex; ey; ez]                [m]
%   u(4:7)   = 베이스 쿼터니언    [qw; qx; qy; qz]
%   u(8:10)  = 베이스 위치        [x; y; z]                   [m]
%   u(11:13) = 베이스 선속도      [vx; vy; vz]                [m/s]
%   u(14:16) = 베이스 각속도      [wx; wy; wz]                [rad/s]
%   u(17:23) = 관절각             [q1..q7]                    [rad]
%   u(24:30) = 관절각속도         [qdot1..qdot7]              [rad/s]
%   u(31:37) = 관절 토크          [tau1..tau7]                [Nm]
%   u(38)    = RL 액션             [-1, 1]
%   u(39)    = docked              [0|1]
%   u(40)    = 시뮬레이션 시간    [s]  (미사용)
%   u(41)    = 조작성 지표         sqrt(det(J*J')) = prod(sv)
%
% 출력 obs (30×1): 정규화된 관측 벡터
%   obs(1:3)   = EE 위치 오차       (정규화)
%   obs(4:7)   = 베이스 쿼터니언    (그대로, 이미 단위 노름)
%   obs(8:10)  = 베이스 각속도      (정규화)
%   obs(11:13) = 베이스 선속도      (정규화)
%   obs(14:20) = 관절각             (정규화)
%   obs(21:27) = 관절각속도         (정규화)
%   obs(28)    = RL 액션            (그대로)
%   obs(29)    = docked             (0 또는 1)
%   obs(30)    = 조작성 지표        (정규화)

    %% 입력 파싱
    e_ee     = u(1:3);
    q_base   = u(4:7);      % quat [qw qx qy qz]
    p_base   = u(8:10);
    w_base   = u(11:13);
    v_base   = u(14:16);
    q_arm    = u(17:23);
    qdot_arm = u(24:30);
    % u(31:37): 토크 — 미사용
    action   = u(38);
    docked   = u(39);
    time     = u(40);% 시간 — 미사용

    %% 정규화 스케일 (임시 — 튜닝 필요)
    s_ee  = 0.015;    % EE 위치 오차  [m]        ~10 cm 기준
    s_pb  = 0.5;    % 베이스 위치   [m]
    s_vb  = 0.0055;   % 베이스 선속도 [m/s]
    s_wb  = 0.01;   % 베이스 각속도 [rad/s]
    s_q   = [3;3;3;3;3;5;3];     % 관절각        [rad]
    s_qd  = 0.18;    % 관절각속도    [rad/s]
    s_m   = 5.0;    % 조작성 지표 정규화 — 정상범위 3~9 기준 중간값
    s_t   = 210;


    %% 관측 벡터 구성 (30×1)
    obs = [
        e_ee     / s_ee;    % (3×1)  obs(1:3)   EE 위치 오차
        q_base;             % (4×1)  obs(4:7)   베이스 쿼터니언 (단위 노름)
        % p_base   / s_pb;    % (3×1)  obs(8:10)  베이스 위치
        w_base   / s_wb;    % (3×1)  obs(8:10)  베이스 각속도
        v_base   / s_vb;    % (3×1)  obs(11:13) 베이스 선속도
        q_arm   ./ s_q;     % (7×1)  obs(14:20) 관절각
        qdot_arm / s_qd;    % (7×1)  obs(21:27) 관절각속도
        action;             % (1×1)  obs(28)    RL 액션
        docked;             % (1×1)  obs(29)    docked 여부
        time    / s_t
    ];   % 30×1
end
