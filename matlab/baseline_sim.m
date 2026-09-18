%% ===== 베이스라인 시뮬레이션 (RL 없이 순수 QP 성능 확인) =====

% addpath("C:\Junesuk\Graduate\Research\0_SMS\3dofSim\ROBOT_sims\simulink-agentic-toolkit")  % (old laptop / GPU server via junction)
% satk_initialize

%clc; close all; clear;
clear;
addpath(genpath(pwd));

loaded_data = load('concept3_for_sim.mat');
robot1 = loaded_data.robot;
robot1.DataFormat = 'column';

%% ===== 초기 조건 설정 =====
port_num       = 10;
sat_docked_num = 6;
%obs_docked_num = 9;

port_target = AB2port(port_num);
sat_docked  = AB2port(sat_docked_num);
%docked      = AB2port(obs_docked_num);

[opa, mass] = set_obs([0 0 0 0 0  0 0 0 1 0]);
obs_vec = opa;
obs_vec(sat_docked_num) = 1;

port = port_pos();

q0b   = [1;0;0;0;0;0;0];
q0arm = rad2deg([-1.283; -1.622; 2.2761853; -1.8021853; 1.112; 1.859; 0.05328]); %q0arm = [-90;-90;160;-130;60;-90;0];

q0    = [q0b; deg2rad(q0arm)];

sim_state = 1;

T_RL = 1;
Tlog = 0.1;
Tf = 220;
Tq = 0.1;

use_RL = 1;
if use_RL
    load('good_agent.mat');
end
%% ===== 시뮬레이션 실행 =====
mdl = "concept3_simulink_RL";
%open_system(mdl);

agentBlk = mdl + "/RL/RL Agent";
if use_RL
    set_param(agentBlk, 'Commented', 'off');
else
    set_param(agentBlk, 'Commented', 'on');
end
% set_param(mdl, 'FastRestart', 'on');

% RL ResetFcn 대신 워크스페이스 변수 직접 주입
in = Simulink.SimulationInput(mdl);
in = setVariable(in, 'q0', q0, 'Workspace', mdl);

out = sim(in);

%% ===== 결과 분석 =====
% To Workspace → Timeseries 형식: .Data (n x 1 x N), .Time (N x 1)
%{
state_data  = squeeze(out.state.Data);    % 14 x N
torque_data = squeeze(out.torque.Data);   % 7  x N
tt          = out.state.Time;             % N  x 1

xbq_end = state_data(:, end);   % 14x1, 마지막 샘플

% 토크: 7 x N (squeeze 후 이미 올바른 형태)
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
%}
plot_results(out)

%% ===== 로컬 함수 =====
function port = AB2port(n)
    port_pos_data = [
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
    port = port_pos_data(n,:);
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
    mass    = 40;
    opacity = port_vector;
    solid_mass = port_vector * mass;
end
