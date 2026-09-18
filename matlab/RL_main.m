%% ===== concept3 TD3 강화학습 메인 스크립트 =====
% 목적  : null-space alpha를 TD3로 학습
%         → 베이스 반동 최소화 (homing 임무)
%
% 액션  : Δα ∈ [-1, 1]  (1×1 스칼라)
% 관측  : 29×1  ← RL_obs 출력
% 보상  : RL_r  (베이스속도 + 베이스자세 + 액션smooth + 생존)
%
% Simulink 연결:
%   - RL Agent 블럭 : concept3_simulink_RL/RL/RL Agent
%   - 관측(29×1)    : RL_obs 출력
%   - 보상(스칼라)  : RL_r 출력 r_tot
% ─────────────────────────────────────────────────────────────

clc; close all; clear;
previousRngState = rng(0, 'twister');
addpath(genpath(pwd));

use_RL =1 ;

%% ===== 초기 조건 설정 =====
port_num       = 10;
sat_docked_num = 6;

port_target = AB2port(port_num);
sat_docked  = AB2port(sat_docked_num);

[opa, mass]  = set_obs([0 0 0 0 0  0 0 0 1 0]);
obs_vec   = opa;
obs_vec(sat_docked_num) = 1;

port = port_pos();

q0b   = [1; 0; 0; 0; 0; 0; 0];
q0arm = rad2deg([-1.283; -1.622; 2.2761853; -1.8021853; 1.112; 1.859; 0.05328]);  % [rad]
q0    = [q0b; q0arm];

sim_state = 1;

%% ===== 시간 설정 =====
T_RL = 2;      % RL 결정 주기 [s]
Tlog = 0.1;    % 로깅 주기 [s]
Tf   = 210;    % 에피소드 길이 [s]
Tq   = 0.1;    % 물리 스텝 [s]

%% ===== Simulink 모델 열기 =====
mdl = "concept3_simulink_RL";
open_system(mdl);
set_param(mdl, 'FastRestart', 'off');
agentBlk = mdl + "/RL/RL Agent";
% 학습 중 불필요 로그 비활성화
set_param(mdl, 'SaveOutput',    'off');
set_param(mdl, 'SaveState',     'off');
set_param(mdl, 'SaveTime',      'off');
set_param(mdl, 'SaveFinalState','off');
set_param(mdl, 'SignalLogging', 'off');
set_param(mdl, 'LoggingToFile', 'off');
set_param(mdl, 'MatFileLogging','off');
set_param(mdl, 'StreamToWks',   'off');
set_param(agentBlk, 'Commented', 'off'); 
save_system(mdl);

set_param(mdl, 'FastRestart', 'on');

%% ===== 관측 / 액션 정보 =====
obsDim  = 30;   % RL_obs 출력 (3+4+3+3+7+7+1+1)
obsInfo = rlNumericSpec([obsDim 1], ...
    'LowerLimit', -5*ones(obsDim, 1), ...
    'UpperLimit',  5*ones(obsDim, 1), ...
    'Name', 'obs');

actInfo = rlNumericSpec([1 1], ...
    'LowerLimit', -1, ...
    'UpperLimit',  1, ...
    'Name', 'delta_alpha');

%% ===== 환경 생성 =====
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

env.ResetFcn = @(in) localResetFcn(in, mdl, q0);

%% ===== TD3 에이전트 생성 =====
agent = rlTD3Agent(obsInfo, actInfo);

agent.AgentOptions.SampleTime = T_RL;

% Critic 1
agent.AgentOptions.CriticOptimizerOptions(1).LearnRate             = 2e-3;
agent.AgentOptions.CriticOptimizerOptions(1).GradientThreshold     = 1;
agent.AgentOptions.CriticOptimizerOptions(1).L2RegularizationFactor = 1e-4;

% Critic 2
agent.AgentOptions.CriticOptimizerOptions(2).LearnRate             = 2e-3;
agent.AgentOptions.CriticOptimizerOptions(2).GradientThreshold     = 1;
agent.AgentOptions.CriticOptimizerOptions(2).L2RegularizationFactor = 1e-4;

% Actor
% c2 대비 concept3: obs 29D(↑), 에피소드 2.6x 길어짐 → LR c2와 동일하게 유지
agent.AgentOptions.ActorOptimizerOptions.LearnRate              = 1e-3; % 5e-4
agent.AgentOptions.ActorOptimizerOptions.GradientThreshold      = 1;
agent.AgentOptions.ActorOptimizerOptions.L2RegularizationFactor = 1e-6;

% 버퍼 / 배치
% c2: MiniBatch=512, Buffer=1e5 / concept3: 에피소드 길어 경험 빠르게 채워짐 → 동일
agent.AgentOptions.ExperienceBufferLength              = 1e5;
agent.AgentOptions.MiniBatchSize                       = 512;
% DiscountFactor: c2=0.99(100 steps) → concept3=0.998(260 steps)
%   γ^260=0.60 으로 에피소드 끝 보상이 충분히 전파
% T_RL=2 기준: γ_per_sec 유지 → 0.998^2 ≈ 0.996, γ^105 = 0.65
agent.AgentOptions.DiscountFactor                      = 0.996;
agent.AgentOptions.ResetExperienceBufferBeforeTraining = true;
% T_RL=2 → 105 steps/ep: 에피소드 비율 유지 (20/210 ≈ 10/105)
agent.AgentOptions.NumStepsToLookAhead                 = 10;

% TD3 파라미터: c2=1e-3, concept3는 3D 복잡도 고려해 중간값
agent.AgentOptions.TargetSmoothFactor = 5e-3;

agent.AgentOptions.TargetPolicySmoothModel.Variance            = 0.1;

%% ===== 학습 옵션 =====
% c2: 5000ep × 100step = 50만 / concept3: 3000ep × 260step = 78만 총 스텝
maxEpisodes = 15000;
maxSteps    = round(Tf / T_RL);   % 260 steps/episode

% 탐색 노이즈 — 전체 스텝의 noise_decay_pct 비율에서 σ_min 도달하도록 자동 계산
% rate = 1 - (σ_min/σ₀)^(1/n_target)
noise_decay_pct = 0.80;   % 추천 0.60~0.80 (70%: 탐색/활용 균형)
sigma_0         = 0.4;   % T_RL=2에서 0.7은 과도한 흔들림 → 0.4로 감소
sigma_min       = 0.05;
n_target        = noise_decay_pct * maxEpisodes * maxSteps;
noise_decay_rate = 1 - (sigma_min / sigma_0)^(1 / n_target);
fprintf('노이즈 감쇠율: %.3e  (%.0f%%@%d ep)\n', ...
        noise_decay_rate, noise_decay_pct*100, maxEpisodes);  % 원래 5천에피추천 5e-6

agent.AgentOptions.ExplorationModel.Variance                   = sigma_0;
agent.AgentOptions.ExplorationModel.StandardDeviationDecayRate = noise_decay_rate;
agent.AgentOptions.ExplorationModel.StandardDeviationMin       = sigma_min;

trainOpts = rlTrainingOptions( ...
    MaxEpisodes             = maxEpisodes, ...
    MaxStepsPerEpisode      = maxSteps, ...
    ScoreAveragingWindowLength = 50, ...
    Verbose                 = false, ...
    Plots                   = 'training-progress', ...
    StopTrainingCriteria    = 'None', ...
    SaveAgentCriteria       = 'None');

trainOpts.UseParallel = true;
trainOpts.ParallelizationOptions.Mode                 = 'async';
% T_RL=2 → 105 steps/ep: 32 → 16
trainOpts.ParallelizationOptions.StepsUntilDataIsSent = 16;

try
    trainOpts.ParallelizationOptions.WorkerRandomSeeds = 'auto';
catch
end

%% ===== 학습 / 에이전트 로드 =====
evaluator = rlEvaluator(NumEpisodes=1, EvaluationFrequency=50);

doTraining = true;

if doTraining
    trainingResults = train(agent, env, trainOpts, Evaluator=evaluator);
else
    agentFile = 'concept3_RL_agent.mat';
    if isfile(agentFile)
        S     = load(agentFile, 'agent');
        agent = S.agent;
        disp('에이전트 로드 완료');
    else
        warning('에이전트 파일 없음. 새 에이전트로 진행합니다.');
    end
end

%% ===== 에이전트 저장 =====
timestamp  = char(datetime('now', 'Format', 'MMdd_HHmm'));
agentSave  = ['concept3_RL_agent_' timestamp '.mat'];
save(agentSave, 'agent', '-v7.3');
fprintf('에이전트 저장: %s\n', agentSave);

%% ===== 검증 시뮬레이션 =====
try
    agent.AgentOptions.ExplorationModel.Variance    = 0;
    agent.AgentOptions.ExplorationModel.VarianceMin = 0;
catch
end

simOpts    = rlSimulationOptions('MaxSteps', maxSteps);
experience = sim(env, agent, simOpts);
fprintf('검증 총 보상: %.4f\n', sum(experience.Reward));

rng(previousRngState);

%% ===== 후처리: 최종 물리 시뮬레이션 =====
disp('=== 최종 시뮬레이션 실행 ===');
set_param(mdl, 'FastRestart', 'off');
set_param(mdl, 'SaveOutput',    'on');
set_param(mdl, 'SaveTime',      'on');
set_param(mdl, 'SignalLogging', 'on');
set_param(mdl, 'MatFileLogging','on');
set_param(mdl, 'StreamToWks',   'on');

in = Simulink.SimulationInput(mdl);
in = setVariable(in, 'q0', q0, 'Workspace', mdl);
out = sim(in);

%% ===== 결과 분석 =====
%{
state_data  = squeeze(out.state.Data);    % 14 x N
torque_data = squeeze(out.torque.Data);   % 7  x N
tt          = out.state.Time;             % N  x 1

xbq_end = state_data(:, end);   % 14x1

% 토크 적분 지표
Torque         = torque_data;
Torque_norm    = vecnorm(Torque, 2, 1);
Torque_L1_norm = sum(abs(Torque), 1);
J_L1           = trapz(tt, Torque_L1_norm);
T_RMS_total    = sqrt(trapz(tt, Torque_norm.^2) / tt(end));

fprintf('▶ Total Joint Torque L1 norm = %.4f [N·m·s]\n', J_L1);
fprintf('▶ Total Joint Torque RMS     = %.4f [N·m]\n',   T_RMS_total);

% 베이스 자세 오차
q0_base   = [1 0 0 0];
qe_base   = xbq_end(1:4);
theta_deg = rad2deg(2*acos(dot(q0_base, qe_base) / ...
                   (norm(q0_base)*norm(qe_base))));
fprintf('▶ Final Base Orientation Error = %.3f deg\n', theta_deg);

% 베이스 위치 오차
pos_final = xbq_end(5:7);
fprintf('▶ Final Base Position Drift   = %.4f m\n', norm(pos_final));
%}
plot_results(out)

%% ===== 로컬 함수 =====
function in = localResetFcn(in, mdl, q0)
    in = setVariable(in, 'q0', q0, 'Workspace', mdl);
end

function port = AB2port(n)
    data = [
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
    port = data(n, :);
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
    mass       = 40;
    opacity    = port_vector;
    solid_mass = port_vector * mass;
end
