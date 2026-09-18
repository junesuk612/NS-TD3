function [r_tot, r] = RL_r2(u)
%#codegen
% RL_r2  보상 위주 설계 — EE추종·베이스속도·베이스자세를 양수 보상으로
%
% 입력 u (41×1): RL_r.m과 동일
%   u(1:3)   = EE 오차            [ex; ey; ez]       [m]
%   u(4:7)   = 베이스 쿼터니언    [qw; qx; qy; qz]
%   u(11:13) = 베이스 선속도      [vx; vy; vz]       [m/s]
%   u(14:16) = 베이스 각속도      [wx; wy; wz]       [rad/s]
%   u(38)    = RL 액션            [-1, 1]
%   u(39)    = docked             [0|1]
%   u(40)    = 시뮬레이션 시간   [s]
%
% 출력:
%   r_tot  (스칼라) : 총 보상
%   r      (5×1)   : [r_track; r_recoil; r_base; r_smooth; r_limit]
%
% 스텝당 범위 (r_scale=0.5 이후):
%   최대 (θ=0, t=210s): 0.5*(1.0 + 1.0 + 4.0 + 0 + 0)   = +3.0
%   베이스라인 (θ=7.4°): r_base=0, r_track+r_recoil 위주
%   θ>7.4° 시 r_base < 0 (무제한 패널티)

    persistent a_prev
    if isempty(a_prev); a_prev = 0; end

    %% 입력 파싱
    e_ee   = u(1:3);
    q_base = u(4:7);
    w_base = u(14:16);
    q2     = u(18);    % 2번 관절각 (QP 위치 제약 대상)
    a_cur  = u(38);
    docked = u(39);
    t      = u(40);

    %% 정규화 스케일
    s_ee = 0.015;   % EE 위치 오차 기준 [m]   (~1.5cm 이내면 exp > 1/e)
    s_vb = 0.005;   % 베이스 선속도 기준 [m/s]
    s_wb = 0.005;    % 베이스 각속도 기준 [rad/s]
    s_s  = 0.3;     % 액션 변화율 기준

    %% 1. EE 추종 보상 — 오차 작을수록 큰 양수 (Gaussian)
    % 최대 w_track (e=0), e=s_ee 시 약 37%로 감소
    w_track = 0.5;
    r_track = w_track * exp(-sum((e_ee / s_ee).^2));

    %% 2. 베이스 속도 보상 — 선속도·각속도 모두 작을수록 양수
    % 각속도 위주 (선속도는 1/4 가중)
    w_recoil = 0.0*1.0;
    vel_sq   = sum((w_base / s_wb).^2);
    r_recoil = w_recoil * exp(-vel_sq);

    %% 3. 베이스 자세 보상 — 시변 가중치, 선형 형태
    % θ_ref(=7.4°, 베이스라인)에서 0, θ=0°에서 최대 +w_bth, θ>θ_ref면 패널티
    % 선형: 클립 없이 항상 일정한 gradient → 개선량에 정비례
    qw        = q_base(1);
    theta_err = 2 * acos(min(abs(qw), 1.0));   % 회전각 오차 [rad]
    theta_ref = deg2rad(6);                   % RL 없을 때 복귀 자세 (베이스라인)

    t0    = 120.0;
    Tf_ep = 210.0;
    w_min = 0.5;
    w_max = 7.0;
    if t < t0
        w_bth = w_min;
    else
        ramp  = ((t - t0) / (Tf_ep - t0))^2;   % 0→1 quadratic
        w_bth = w_min + (w_max - w_min) * ramp;
    end
    r_base = w_bth * (1.0 - theta_err / theta_ref);

    %% 3-1. 최종 자세 터미널 보너스 (마지막 스텝에만 1회 지급)
    % T_RL=2s 기준: t≥208s → 마지막 스텝
    % θ=0°: +15, θ=θ_ref(7.4°): 0, θ>θ_ref: 음수 (클립 없음)
    % 크기: 스텝당 r_base 최대(5.0)의 3배 → 최종 자세를 명확히 차별화
    if t >= Tf_ep - 2.0
        w_terminal = 5.0;
        r_terminal = w_terminal * (1.0 - theta_err / (theta_ref/2));
        r_base     = r_base + r_terminal;
    end

    %% 4. 액션 변화율 패널티 — 급격한 α 변화 억제

    % 클립 없음: change=1.0 → -11, change=2.0 → -44 (r_base 최대 +3.5보다 훨씬 큼)
    w_sm = 1;
    r_smooth = -w_sm * ((a_cur - a_prev) / s_s)^2;
    a_prev   = a_cur;

    %% 5. q2 관절 한계 + 방향성 액션 패널티
    % 부호 규칙: 양수 alpha → q2 감소 → 하한(-150°) 방향
    %            음수 alpha → q2 증가 → 상한(-45°) 방향
    % 한계 근처에서 한계 방향으로 미는 액션만 제곱 패널티
    % (반대 방향 또는 alpha=0은 패널티 없음 → 탈출 허용)
    q2_min_lim = deg2rad(-135);
    q2_max_lim = deg2rad(-45);
    margin_r   = deg2rad(20);
    w_limit    = 2.0;

    if q2 < q2_min_lim + margin_r           % 하한 근접
        proximity  = (q2_min_lim + margin_r - q2) / margin_r;   % 1=한계, 0=경계
        bad_action = max(0,  a_cur);         % 양수 alpha = 하한 방향 = 낭비
        r_limit    = -w_limit * proximity * bad_action^2;
    elseif q2 > q2_max_lim - margin_r       % 상한 근접
        proximity  = (q2 - (q2_max_lim - margin_r)) / margin_r;
        bad_action = max(0, -a_cur);         % 음수 alpha = 상한 방향 = 낭비
        r_limit    = -w_limit * proximity * bad_action^2;
    else
        r_limit = 0;
    end

    %% 도킹 후 EE 추종 비중 감소
    if docked > 0.5
        r_track = r_track * 0.2;
    end

    %% 합산 및 스케일
    r_alive = 1;

    r_scale = 0.5;
    r_raw   = r_track + r_recoil + r_base + r_smooth + r_limit + r_alive;

    r     = r_scale * [r_track; r_recoil; r_base; r_smooth; r_limit ;r_alive];
    r_tot = r_scale * r_raw;
end
