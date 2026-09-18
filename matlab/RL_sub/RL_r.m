function [r_tot, r] = RL_r(u)
%#codegen
% RL_r  concept3 강화학습 보상 함수
%
% 입력 u (40×1):
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
%   u(40)    = 시뮬레이션 시간    [s]  (Clock 블록)
%   u(41)    = 최소 특이값          min(svd(Jg))   [수도인버스 측에서 전달]
%
% 출력:
%   r_tot  (스칼라) : 총 보상
%   r      (6×1)   : [r_track; r_recoil; r_base; r_smooth; r_alive; r_manip]
    
    %% 이전 액션 저장 (smooth 패널티용)
    persistent a_prev
    if isempty(a_prev); a_prev = 0; end

    %% 입력 파싱
    e_ee     = u(1:3);
    q_base   = u(4:7);      % [qw qx qy qz]
    w_base   = u(11:13);
    v_base   = u(14:16);
    a_cur    = u(38);
    docked   = u(39);
    t        = u(40);
    %m_manip  = u(41);       % Yoshikawa 조작성 지표

    %% 정규화 스케일 (임시 — 튜닝 필요)
    s_ee  = 0.015;    % EE 위치 오차  [m]
    s_vb  = 0.0055;   % 베이스 선속도 [m/s]
    s_wb  = 0.01;
    s_s   = 0.3;    % 액션 변화율 정규화

    %% 1. EE 위치 추종 오차 (주 목표)
    w_track    = 4.0;
    clip_track = -10.0*1000;
    r_track = max(-w_track * sum((e_ee / s_ee).^2), clip_track);

    %% 2. 베이스 순간 속도 페널티 (반동 억제)
    % isdone 제거 후 폭발 에피소드가 끝까지 실행되므로 Critic 안정화를 위해 클립 필수
    w_vb    = 0.0;
    w_wb    = 1.5;
    r_recoil = -w_vb * sum((v_base / s_vb).^2) ...
               -w_wb * sum((w_base / s_wb).^2);
    r_recoil = max(r_recoil, -15*1000);   % 플롯 기준 -10 (정상 상한의 ~3배)

    %% 3. 베이스 자세 편차 페널티 (ramp 가중치)
    % qw=1이 항등: (1 - |qw|) ∝ 회전각, s_thb로 정규화
    s_thb     = 0.05;   % 정규화 기준 [rad] — 약 3° 이탈 시 정규화값=1
    ramp_lin  = max(0, min((t - 120.0) / (200.0 - 120.0), 1.0));
    ramp      = ramp_lin^3;          % 2차 convex: 초반 완만 → 후반 급격히 증가
    w_bth     = 3 * ramp;
    qw     = q_base(1);
    theta_err = 2 * acos(min(abs(qw), 1.0));      % [0, π] 회전각 오차 [rad]
    r_base = -w_bth * (theta_err / s_thb)^2;
    r_base = max(r_base, -20*1000);  % 플롯 기준 -20 (ramp 최대 구간 폭발 억제)

    %% 4. 액션 변화율 페널티 (부드러운 출력)
    w_s      = 50;
    w_c      = -10*1000;
    % r_smooth = -w_s * ((a_cur - a_prev) / s_s)^2;
    r_smooth = max(-w_s * ((a_cur - a_prev) / s_s)^2, w_c);
    
    a_prev   = a_cur;

    %% 5. 생존 보너스
    r_alive = 4.0;


    %% 도킹 후 보상 비중 전환 (임시: docked=1이면 추종 오차 비중 감소)
    if docked > 0.5
        r_track = r_track * 0.2;   % 도킹 후 EE 추종 중요도 낮춤
    end

    %% 합산 및 스케일
    r_scale = 0.5;   % 임시 스케일 — 튜닝 필요
    r_raw   = r_track + r_recoil + r_base + r_smooth + r_alive;

    r     = r_scale * [r_track; r_recoil; r_base; r_smooth; r_alive];
    r_tot = r_scale * r_raw;
end
