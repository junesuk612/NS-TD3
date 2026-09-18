function alpha_out = alpha_star_lookup(clock)
%#codegen
% offline α*(t) 룩업 — Simulink MATLAB Function 블록
%
% 입력:
%   clock     (1×1): 시뮬레이션 절대 시간 [s]
%
% 출력:
%   alpha_out (1×1): null-space 파라미터 α*(t)
%
% 동작:
%   presim에서 계산된 전체 구간(0~Tf) α*(t)를 clock 기준으로 룩업

persistent alpha_data Ts_inv N_data t0_data

if isempty(alpha_data)
    raw        = coder.load('alpha_star_offline.mat', 'alpha_star', 'tt');
    alpha_data = raw.alpha_star;                                          % N×1
    Ts_inv     = (length(raw.tt) - 1) / (raw.tt(end) - raw.tt(1));      % 1/Ts
    N_data     = int32(length(raw.alpha_star));
    t0_data    = raw.tt(1);
end

% clock 기준 직접 인덱싱 (균일 시간축 가정)
idx = int32(round((clock - t0_data) * Ts_inv)) + int32(1);
idx = max(int32(1), min(N_data, idx));
alpha_out = alpha_data(idx);
end
