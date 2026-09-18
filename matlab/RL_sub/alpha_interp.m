function alpha_out = alpha_interp(alpha_rl, t)
%#codegen
% RL 액션 보간기 — 위치 / 속도 모드 전환
%
% [MODE = 1] 위치 모드 (기존)
%   alpha_rl = RL이 출력한 alpha 목표값 ∈ [-1, 1]
%   T_RL 구간에 걸쳐 이전 값 → 새 값으로 선형 보간
%   출력: 부드럽게 보간된 alpha (연속)
%
% [MODE = 2] 속도 모드
%   alpha_rl = RL이 출력한 d(alpha)/dt ∈ [-1, 1]  [1/s 단위]
%   매 Tq 호출마다 Euler 적분 → alpha 누적
%   출력: 적분된 alpha ∈ [-alpha_max, alpha_max] (연속 ramp, 점프 없음)
%   장점: 구조적으로 smooth — RL이 급변해도 alpha는 완만하게 변함
%
% 모드 전환: 아래 MODE 값만 수정
% ──────────────────────────────────────────────────────

    MODE      = 2;      % ← 1(위치) 또는 2(속도)  ★ 여기만 바꾸면 됨
    Ts_RL     = 2.0;    % RL 결정 주기 [s]  (위치 모드: 보간 시간)
    alpha_max = 1.0;    % 속도 모드: 적분 포화 한계

% ──────────────────────────────────────────────────────

    persistent alpha_start alpha_end t_change   % 위치 모드용
    persistent alpha_integ t_prev               % 속도 모드용

    %% 초기화 (에피소드 시작 또는 첫 호출)
    if isempty(alpha_start)
        alpha_start = 0;
        alpha_end   = alpha_rl;
        t_change    = t;
        alpha_integ = 0;
        t_prev      = t;
        alpha_out   = 0;
        return;
    end

    if MODE == 1
        %% ===== 위치 모드: 선형 보간 (기존 동작 그대로) =====
        if abs(alpha_rl - alpha_end) > 1e-9
            % 새 RL 결정 도착 → 현재 보간 위치에서 다시 시작
            frac        = min((t - t_change) / Ts_RL, 1.0);
            alpha_start = alpha_start + frac * (alpha_end - alpha_start);
            alpha_end   = alpha_rl;
            t_change    = t;
        end
        frac      = min((t - t_change) / Ts_RL, 1.0);
        alpha_out = alpha_start + frac * (alpha_end - alpha_start);

    else
        %% ===== 속도 모드: Euler 적분 =====
        % alpha_rl = d(alpha)/dt, ZOH로 T_RL 동안 유지
        % → 적분하면 T_RL 구간이 선형 ramp, RL 결정 시점에서도 alpha는 연속
        dt          = t - t_prev;
        alpha_integ = alpha_integ + 0.5*alpha_rl * dt; 
        alpha_integ = max(-alpha_max, min(alpha_max, alpha_integ));
        alpha_out   = alpha_integ;
    end

    t_prev = t;   % 두 모드 공통 업데이트
end
