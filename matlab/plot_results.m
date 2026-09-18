function plot_results(out)
% plot_results  시뮬레이션 결과 시각화
%
% 입력 out (sim 출력):
%   out.state   (14×1×N): [qw qx qy qz  x y z  q1..q7]
%   out.torque  ( 7×1×N): 관절 토크 [Nm]
%   out.x_cmd   ( 6×1×N): EE 위치 명령 [x y z roll pitch yaw] (위치[m], 자세[deg])
%   out.b2EE    ( 6×1×N): EE 실제 포즈 [x y z roll pitch yaw] (위치[m], 자세[deg])

%% 데이터 파싱
tt_state    = out.state.Time;
tt_torque   = out.torque.Time;
tt_xcmd     = out.x_cmd.Time;
tt_ee       = out.b2EE.Time;

state_data  = squeeze(out.state.Data);    % 14×N_s
torque_data = squeeze(out.torque.Data);   % 7×N_t
xcmd_data   = squeeze(out.x_cmd.Data);   % 6×N_x
ee_data     = squeeze(out.b2EE.Data);    % 6×N_e

% 베이스 상태
q_base = state_data(1:4, :);   % [qw qx qy qz]
p_base = state_data(5:7, :);   % [m]

% 관절 상태
q_arm  = state_data(14:20, :);  % 관절각 [rad], 7×N
%dt     = mean(diff(tt_state));
% qdot_arm = gradient(q_arm, dt); % 수치 미분으로 관절각속도 추정, 7×N

qdot_arm = state_data(21:27, :);

% EE 실제
ee_pos = ee_data(1:3, :);      % [m]
ee_rpy = ee_data(4:6, :);      % [deg]

% EE 명령 (tt_ee 기준으로 보간)
cmd_pos = interp1(tt_xcmd, xcmd_data(1:3,:)', tt_ee, 'linear', 'extrap')';
cmd_rpy = interp1(tt_xcmd, xcmd_data(4:6,:)', tt_ee, 'linear', 'extrap')';

% 추종 오차
err_pos = cmd_pos - ee_pos;    % [m]
err_rpy = cmd_rpy - ee_rpy;    % [deg]

% 베이스 자세 오차
qw_vec         = q_base(1, :);
theta_rad      = 2 * acos(min(abs(qw_vec), 1.0));
theta_deg_base = rad2deg(theta_rad);

% 토크 노름
torque_norm = vecnorm(torque_data, 2, 1);
torque_l1   = sum(abs(torque_data), 1);
J_L1        = trapz(tt_torque, torque_l1);
T_RMS       = sqrt(trapz(tt_torque, torque_norm.^2) / tt_torque(end));

%% 콘솔 출력
fprintf('=== 성능 요약 ===\n');
fprintf('  Torque L1 norm  : %.4f [N·m·s]\n', J_L1);
fprintf('  Torque RMS      : %.4f [N·m]\n',   T_RMS);
fprintf('  Base Orien. Err : %.3f [deg] (최종)\n', theta_deg_base(end));
fprintf('  Base Pos. Drift : %.4f [m] (최종)\n',   norm(p_base(:, end)));
fprintf('  EE pos RMSE     : x=%.4f  y=%.4f  z=%.4f [m]\n', ...
    rms(err_pos(1,:)), rms(err_pos(2,:)), rms(err_pos(3,:)));
fprintf('  EE rpy RMSE     : r=%.3f  p=%.3f  y=%.3f [deg]\n', ...
    rms(err_rpy(1,:)), rms(err_rpy(2,:)), rms(err_rpy(3,:)));

%% Figure 1: 3D EE 궤적 (명령 + 실제)
figure('Name', '3D EE Trajectory', 'NumberTitle', 'off');
plot3(ee_pos(1,:), ee_pos(2,:), ee_pos(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual'); hold on;
plot3(xcmd_data(1,:), xcmd_data(2,:), xcmd_data(3,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Command');
plot3(ee_pos(1,1),   ee_pos(2,1),   ee_pos(3,1),   'go', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot3(ee_pos(1,end), ee_pos(2,end), ee_pos(3,end), 'bs', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'b', 'DisplayName', 'End (Actual)');
grid on; xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('End-Effector 3D Trajectory');
legend('Location', 'best'); view(45, 30);

%% Figure 2: EE 추종 오차 (위치 + 자세)
figure('Name', 'EE Tracking Error', 'NumberTitle', 'off');
labels_ep = {'e_x [m]', 'e_y [m]', 'e_z [m]'};
labels_er = {'e_{roll} [deg]', 'e_{pitch} [deg]', 'e_{yaw} [deg]'};
for i = 1:3
    subplot(3, 2, 2*i-1);
    plot(tt_ee, err_pos(i,:), 'b', 'LineWidth', 1.2);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel(labels_ep{i}); grid on;
    if i == 1; title('EE Position Error (cmd - actual)'); end
    if i == 3; xlabel('Time [s]'); end

    subplot(3, 2, 2*i);
    plot(tt_ee, err_rpy(i,:), 'r', 'LineWidth', 1.2);
    yline(0, 'k--', 'LineWidth', 0.8);
    ylabel(labels_er{i}); grid on;
    if i == 1; title('EE Orientation Error (cmd - actual)'); end
    if i == 3; xlabel('Time [s]'); end
end

%% Figure 3: EE 명령 vs 실제 (실제 먼저, 명령 점선 위에 중첩)
figure('Name', 'EE Position Cmd vs Actual', 'NumberTitle', 'off');
labels_pos = {'X [m]', 'Y [m]', 'Z [m]'};
labels_rpy = {'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]'};
for i = 1:3
    subplot(3, 2, 2*i-1);
    plot(tt_ee, ee_pos(i,:),  'b',   'LineWidth', 1.5, 'DisplayName', 'Actual'); hold on;
    plot(tt_ee, cmd_pos(i,:), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Cmd');
    ylabel(labels_pos{i}); grid on; legend('Location', 'best');
    if i == 1; title('EE Position'); end
    if i == 3; xlabel('Time [s]'); end

    subplot(3, 2, 2*i);
    plot(tt_ee, ee_rpy(i,:),  'b',   'LineWidth', 1.5, 'DisplayName', 'Actual'); hold on;
    plot(tt_ee, cmd_rpy(i,:), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Cmd');
    ylabel(labels_rpy{i}); grid on; legend('Location', 'best');
    if i == 1; title('EE Orientation'); end
    if i == 3; xlabel('Time [s]'); end
end

%% Figure 4: 관절 토크 (7개 한 그래프 + 토크 노름)
figure('Name', 'Joint Torques', 'NumberTitle', 'off');
colors = lines(7);
subplot(2,1,1);
for i = 1:7
    plot(tt_torque, torque_data(i,:), 'Color', colors(i,:), 'LineWidth', 1.1, ...
        'DisplayName', sprintf('\\tau_%d', i)); hold on;
end
legend('Location', 'best', 'NumColumns', 4);
ylabel('[Nm]'); grid on; title('Joint Torques');

subplot(2,1,2);
plot(tt_torque, torque_norm, 'k', 'LineWidth', 1.5);
ylabel('||\tau||_2 [Nm]'); xlabel('Time [s]'); grid on;
title(sprintf('Torque L2 Norm  (L1=%.2f N·m·s, RMS=%.2f N·m)', J_L1, T_RMS));

%% Figure 5: 베이스 자세 오차 + 위치 드리프트
figure('Name', 'Base State', 'NumberTitle', 'off');

subplot(2,1,1);
plot(tt_state, theta_deg_base, 'm', 'LineWidth', 1.5);
ylabel('[deg]'); grid on;
title(sprintf('Base Orientation Error  (Final: %.3f deg)', theta_deg_base(end)));

subplot(2,1,2);
p_drift = vecnorm(p_base, 2, 1);
plot(tt_state, p_drift, 'k', 'LineWidth', 1.5);
ylabel('[m]'); xlabel('Time [s]'); grid on;
title(sprintf('Base Position Drift (Final: %.4f m)', p_drift(end)));

%% Figure 6: 관절각 + 관절각속도
figure('Name', 'Joint States', 'NumberTitle', 'off');
subplot(2,1,1);
for i = 1:7
    plot(tt_state, rad2deg(q_arm(i,:)), 'Color', colors(i,:), 'LineWidth', 1.1, ...
        'DisplayName', sprintf('q_%d', i)); hold on;
end
legend('Location', 'best', 'NumColumns', 4);
ylabel('[deg]'); grid on; title('Joint Angles');

subplot(2,1,2);
for i = 1:7
    plot(tt_state, rad2deg(qdot_arm(i,:)), 'Color', colors(i,:), 'LineWidth', 1.1, ...
        'DisplayName', sprintf('\\dot{q}_%d', i)); hold on;
end
legend('Location', 'best', 'NumColumns', 4);
ylabel('[deg/s]'); xlabel('Time [s]'); grid on; title('Joint Angular Velocities (numerical)');

end
