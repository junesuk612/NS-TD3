%% compare_baselines.m — 세 베이스라인 비교 시각화 및 논문용 데이터 추출
clc; clear; close all;

%% ══════════════════════════════════════════════════════
%  데이터 로드
% ══════════════════════════════════════════════════════
data_dir = fullfile(fileparts(mfilename('fullpath')), 'FINAL_DATA');

s_MN = load(fullfile(data_dir, 'MN_out3.mat'));  out_MN = s_MN.out;
s_RH = load(fullfile(data_dir, 'RH_out.mat'));  out_RH = s_RH.out;
s_RL = load(fullfile(data_dir, 'RL_out3.mat'));  out_RL = s_RL.out;

labels = {'Min-Norm', 'RH-QP', 'NS-TD3'};
outs   = {out_MN, out_RH, out_RL};

clrs   = {[0.20 0.40 0.80], [0.10 0.65 0.30], [0.85 0.10 0.10]};

lspc   = {'-', '-', '-'};
lw     = 1.6;

%% 파싱
D = cell(3,1);
for m = 1:3
    D{m} = parse_out(outs{m});
end

% MN은 null-space 제어 없음 → α=0 고정
D{1}.alpha(:) = 0;

fpos =  [2 2 8 6];%[2 2 14 5];   % 개별 figure 기본 크기 [cm]

t_dock = 112;   % 도킹 시점 [s]

%% Figure 1 — α(t)
figure('Name', 'alpha(t)', 'Units', 'centimeters', 'Position', fpos);
xline(t_dock, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
hold on;
for m = 1:3
    plot(D{m}.tt_alpha, D{m}.alpha, lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
end
ylim([-0.12 0.12]);
ylabel('\alpha'); xlabel('Time [s]');
%title('\alpha(t) Comparison');
legend('Location', 'best'); grid on;
text(t_dock + 1, -0.09, 'Docked', 'FontSize', 9, 'Color', [0.4 0.4 0.4], 'VerticalAlignment', 'top');

%% Figure 2 — 베이스 자세 오차
figure('Name', 'Base Attitude', 'Units', 'centimeters', 'Position', fpos);
xline(t_dock, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
hold on;
for m = 1:3
    plot(D{m}.tt, D{m}.att_err_deg, lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
end
yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
ylabel('Base Attitude Error [deg]'); xlabel('Time [s]');
%title('Base Attitude Error');
legend('Location', 'best'); grid on;
yl = ylim; text(t_dock + 1,3, 'Docked', ...
     'FontSize',  9, 'Color', [0.4 0.4 0.4], 'VerticalAlignment', 'top');

%% Figure 3 — EE 위치 추종 오차
figure('Name', 'EE Error', 'Units', 'centimeters', 'Position', fpos);
xline(t_dock, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
hold on;
for m = 1:3
    plot(D{m}.tt_ee, D{m}.ee_pos_err * 100, lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
end
ylabel('EE Position Error [cm]'); xlabel('Time [s]');
%title('End-Effector Tracking Error');
legend('Location', 'best'); grid on;
yl = ylim; text(t_dock - 28, yl(1) + 0.92*(yl(2)-yl(1)), 'Docked', ...
     'FontSize', 9, 'Color', [0.4 0.4 0.4], 'VerticalAlignment', 'top');

%% Figure 4 — 계산 시간 (시계열)
figure('Name', 'Comp Time', 'Units', 'centimeters', 'Position', fpos);
xline(t_dock, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
hold on;
for m = 1:3
    plot(D{m}.tt_elapsed(5:end), D{m}.elapsed_ms(5:end), lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', 0.9, 'DisplayName', labels{m});
end
ylabel('Computation Time [ms]'); xlabel('Time [s]');
%title('Step Computation Time (1st step excluded)');
legend('Location', 'best'); grid on;
yl = ylim; text(t_dock + 1, yl(1) + 0.92*(yl(2)-yl(1)), 'Docked', ...
     'FontSize', 9, 'Color', [0.4 0.4 0.4], 'VerticalAlignment', 'top');



%% Figure 5 — 계산 시간 분포 (박스플롯)
figure('Name', 'Comp Time Dist', 'Units', 'centimeters', 'Position', fpos);
for m = 1:3
    draw_boxplot(D{m}.elapsed_ms, m, clrs{m});
    hold on;
end
set(gca, 'XTick', 1:3, 'XTickLabel', labels, 'XLim', [0.4 3.6]);
ylabel('Computation Time [ms]');
title('Computation Time Distribution'); grid on;

%% Figure 6 — 2번 관절 각도
figure('Name', 'Joint 2 Angle', 'Units', 'centimeters', 'Position', fpos);
for m = 1:3
    plot(D{m}.tt, rad2deg(D{m}.q_arm(2,:)), lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
    hold on;
end
xline(t_dock, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(-30,  'k--', 'LineWidth', 0.8, 'DisplayName', 'q2 max limit');
yline(-135, 'k:',  'LineWidth', 0.8, 'DisplayName', 'q2 min limit');
ylabel('Joint 2 Angle [deg]'); xlabel('Time [s]');
%title('Joint 2 Angle Comparison');
legend('Location', 'best'); grid on;
yl = ylim; text(t_dock + 1,-50, 'Docked', ...
     'FontSize', 9, 'Color', [0.4 0.4 0.4], 'VerticalAlignment', 'top');


%legend('Location', 'best'); grid on;
%{
%% Figure 7 — 관절 토크 노름
figure('Name', 'Torque Norm', 'Units', 'centimeters', 'Position', fpos);
for m = 1:3
    plot(D{m}.tt_trq, D{m}.trq_norm, lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
    hold on;
end
ylabel('||\tau||_2 [Nm]'); xlabel('Time [s]');
title('Joint Torque Norm');
legend('Location', 'best'); grid on;

%% Figure 8 — 베이스 각속도 노름
%{
figure('Name', 'Base Omega', 'Units', 'centimeters', 'Position', fpos);
for m = 1:3
    plot(D{m}.tt, D{m}.omega_norm, lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
    hold on;
end
ylabel('||\omega_{base}||_2 [rad/s]'); xlabel('Time [s]');
title('Base Angular Velocity');
legend('Location', 'best'); grid on;
%}
%% Figure 9 — 베이스 쿼터니언 벡터부
%{
figure('Name', 'Base Quat', 'Units', 'centimeters', 'Position', fpos);
qclrs_sub = {[0.8 0.2 0.2],[0.2 0.6 0.2],[0.2 0.2 0.8]};
for m = 1:3
    for qi = 1:3
        p = plot(D{m}.tt, D{m}.q_base(qi+1,:), lspc{m}, ...
                 'Color', clrs{m}*0.5 + qclrs_sub{qi}*0.5, 'LineWidth', 0.9);
        p.HandleVisibility = 'off';
        hold on;
    end
end
yline(0,'k:','LineWidth',0.8,'HandleVisibility','off');
for m = 1:3
    plot(nan, nan, lspc{m}, 'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
end
ylabel('Quaternion'); xlabel('Time [s]');
title('Base Quaternion (q_x, q_y, q_z per method)');
legend('Location', 'best'); grid on;
%}
%% Figure 10 — 누적 토크
figure('Name', 'Cumulative Torque', 'Units', 'centimeters', 'Position', fpos);
for m = 1:3
    Ts = mean(diff(D{m}.tt_trq));
    cumtrq = cumsum(D{m}.trq_norm) * Ts;
    plot(D{m}.tt_trq, cumtrq, lspc{m}, ...
         'Color', clrs{m}, 'LineWidth', lw, 'DisplayName', labels{m});
    hold on;
end
ylabel('Cumulative ||\tau||_2 \cdot dt [Nm\cdots]'); xlabel('Time [s]');
title('Cumulative Torque');
legend('Location', 'best'); grid on;
%}
%% ══════════════════════════════════════════════════════
%  논문용 표 데이터
% ══════════════════════════════════════════════════════
sep = repmat('═',1,85);
fprintf('\n%s\n', sep);
fprintf('%-20s  %12s  %12s  %14s  %14s  %12s\n', ...
        'Method', 'FinalAtt[deg]', 'MeanEE[cm]', 'TotalTrqL1[Nm·s]', 'TotalTrqL2[Nm·s]', 'MeanComp[ms]');
fprintf('%s\n', repmat('─',1,85));

for m = 1:3
    d = D{m};
    Ts_trq = mean(diff(d.tt_trq));

    % 최종 자세 오차 (마지막 1초 평균)
    t_win_att = d.tt >= (d.tt(end) - 1.0);
    final_att = mean(d.att_err_deg(t_win_att));

    % EE 추종 오차 (평균)
    mean_ee = mean(d.ee_pos_err) * 100;   % [cm]

    % 총 토크 (L1, L2 × dt)
    total_trq_L1 = sum(sum(abs(d.torque), 1)) * Ts_trq;
    total_trq_L2 = sum(d.trq_norm) * Ts_trq;

    % 평균/최대 계산 시간
    mean_comp = mean(d.elapsed_ms(5:end));
    max_comp  = max(d.elapsed_ms(5:end));

    fprintf('%-20s  %12.4f  %12.4f  %14.4f  %14.4f  %8.4f (max %.4f)\n', ...
            labels{m}, final_att, mean_ee, total_trq_L1, total_trq_L2, mean_comp, max_comp);
end
fprintf('%s\n', sep);

%% ══════════════════════════════════════════════════════
%  로컬 함수
% ══════════════════════════════════════════════════════
function D = parse_out(out)

%% state (27×N): [qw qx qy qz | x y z | vx vy vz | wx wy wz | q_arm(7) | qdot_arm(7)]
state_raw = squeeze(out.state.Data);
if size(state_raw,1) ~= 27; state_raw = state_raw'; end
D.tt       = out.state.Time;
D.q_base   = state_raw(1:4, :);    % [qw; qx; qy; qz]
D.p_base   = state_raw(5:7, :);
D.v_lin    = state_raw(8:10, :);
D.v_ang    = state_raw(11:13, :);  % base angular velocity [rad/s]
D.q_arm    = state_raw(14:20, :);
D.qdot_arm = state_raw(21:27, :);

% 베이스 자세 오차: 2·acos(|qw|) [deg]
qw = D.q_base(1,:);
D.att_err_deg = 2 * acos(min(1, abs(qw))) * 180/pi;

% 베이스 각속도 노름
D.omega_norm = sqrt(sum(D.v_ang.^2, 1));

%% torque (7×N)
trq_raw = squeeze(out.torque.Data);
if size(trq_raw,1) ~= 7; trq_raw = trq_raw'; end
D.tt_trq  = out.torque.Time;
D.torque  = trq_raw;
D.trq_norm = sqrt(sum(trq_raw.^2, 1));

%% alpha
D.alpha    = squeeze(out.alpha.Data);
D.tt_alpha = out.alpha.Time;

%% elapsed [ms] — 첫 스텝 제거 (JIT/초기화로 인한 이상값)
elapsed_raw  = squeeze(out.elapsed.Data) * 1000;
tt_elapsed   = out.elapsed.Time;
D.elapsed_ms = elapsed_raw(2:end);
D.tt_elapsed = tt_elapsed(2:end);

%% EE 추종 오차
b2EE_raw = squeeze(out.b2EE.Data);
if size(b2EE_raw,1) ~= 6; b2EE_raw = b2EE_raw'; end
D.tt_ee = out.b2EE.Time;

xcmd_raw = squeeze(out.x_cmd.Data);
if size(xcmd_raw,1) ~= 6; xcmd_raw = xcmd_raw'; end
tt_xcmd  = out.x_cmd.Time;

% x_cmd를 b2EE 시간축에 보간
xcmd_i = interp1(tt_xcmd, xcmd_raw', D.tt_ee, 'linear', 'extrap')';
D.ee_pos_err = sqrt(sum((b2EE_raw(1:3,:) - xcmd_i(1:3,:)).^2, 1));
end

function draw_boxplot(data, x, clr)
% Statistics Toolbox 없이 박스플롯 직접 그리기
data = data(isfinite(data));
if isempty(data); return; end

q1  = quantile_manual(data, 0.25);
q2  = quantile_manual(data, 0.50);
q3  = quantile_manual(data, 0.75);
iqr_val = q3 - q1;
wlo = max(min(data), q1 - 1.5*iqr_val);
whi = min(max(data), q3 + 1.5*iqr_val);
out_pts = data(data < wlo | data > whi);

% IQR이 거의 0일 때 (분포가 좁을 때) 박스가 사라지는 문제 방지
% 전체 데이터 범위의 2%, 또는 최소 0.001ms 중 큰 값으로 최소 높이 보장
min_box_h = max(0.001, (max(data) - min(data)) * 0.02);
if (q3 - q1) < min_box_h
    q1_vis = q2 - min_box_h / 2;
    q3_vis = q2 + min_box_h / 2;
else
    q1_vis = q1;
    q3_vis = q3;
end

bw = 0.3;
patch([x-bw x+bw x+bw x-bw], [q1_vis q1_vis q3_vis q3_vis], clr, ...
      'FaceAlpha', 0.4, 'EdgeColor', clr, 'LineWidth', 1.2);
plot([x-bw x+bw], [q2 q2], '-', 'Color', clr, 'LineWidth', 2.0);
plot([x x], [wlo q1_vis], '-', 'Color', clr, 'LineWidth', 1.0);
plot([x x], [q3_vis whi], '-', 'Color', clr, 'LineWidth', 1.0);
plot([x-bw*0.5 x+bw*0.5], [wlo wlo], '-', 'Color', clr, 'LineWidth', 1.0);
plot([x-bw*0.5 x+bw*0.5], [whi whi], '-', 'Color', clr, 'LineWidth', 1.0);
if ~isempty(out_pts)
    plot(x*ones(size(out_pts)), out_pts, 'o', ...
         'MarkerSize', 3, 'Color', clr, 'MarkerFaceColor', clr);
end
end

function q = quantile_manual(x, p)
% 기본 quantile (Statistics Toolbox 불필요)
x = sort(x(:));
n = length(x);
idx = p * (n - 1) + 1;
lo  = floor(idx);
hi  = ceil(idx);
if lo == hi
    q = x(lo);
else
    q = x(lo) + (idx - lo) * (x(hi) - x(lo));
end
end
