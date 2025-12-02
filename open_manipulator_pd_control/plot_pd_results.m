% MATLAB Script to plot PD Controller results
% This script reads the log file and creates plots for analysis

clear all; close all; clc;

% Prompt user for log file
[filename, pathname] = uigetfile('*.txt', 'Select PD Control Log File');
if isequal(filename,0)
   disp('User canceled file selection');
   return;
end

% Read the log file
filepath = fullfile(pathname, filename);
fileID = fopen(filepath, 'r');

% Skip header lines (lines starting with #)
tline = fgetl(fileID);
while startsWith(tline, '#')
    disp(tline);  % Display header info
    tline = fgetl(fileID);
end

% Reset file pointer
fclose(fileID);

% Read data (skip lines starting with #)
data = readmatrix(filepath, 'CommentStyle', '#');

% Extract columns
time = data(:, 1);          % Time (s)
reference = data(:, 2);     % Reference position (rad)
current = data(:, 3);       % Current position (rad)
error = data(:, 4);         % Position error (rad)
effort = data(:, 5);        % Control effort

% Create figure with subplots
figure('Position', [100, 100, 1200, 800]);

% Subplot 1: Position tracking
subplot(3, 1, 1);
plot(time, reference, 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
hold on;
plot(time, current, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Current Position');
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Position (rad)', 'FontSize', 12);
title('Position Tracking', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
xlim([min(time), max(time)]);

% Subplot 2: Tracking error
subplot(3, 1, 2);
plot(time, error, 'g-', 'LineWidth', 1.5);
hold on;
plot([min(time), max(time)], [0, 0], 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Error (rad)', 'FontSize', 12);
title('Position Error', 'FontSize', 14, 'FontWeight', 'bold');
xlim([min(time), max(time)]);

% Add error statistics
mean_error = mean(abs(error));
max_error = max(abs(error));
text(0.7*max(time), max(error)*0.8, ...
     sprintf('Mean |Error|: %.4f rad\nMax |Error|: %.4f rad', ...
     mean_error, max_error), ...
     'FontSize', 10, 'BackgroundColor', 'white');

% Subplot 3: Control effort
subplot(3, 1, 3);
plot(time, effort, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Effort', 'FontSize', 12);
title('Control Effort', 'FontSize', 14, 'FontWeight', 'bold');
xlim([min(time), max(time)]);

% Overall title
sgtitle('PD Controller Performance Analysis', 'FontSize', 16, 'FontWeight', 'bold');

% Calculate and display performance metrics
fprintf('\n=== Performance Metrics ===\n');
fprintf('Settling time (2%% criterion): ');
settling_error_threshold = 0.02 * abs(reference(end) - reference(1));
settling_idx = find(abs(error) <= settling_error_threshold, 1, 'first');
if ~isempty(settling_idx)
    settling_time = time(settling_idx);
    fprintf('%.3f s\n', settling_time);
else
    fprintf('Not achieved\n');
end

fprintf('Overshoot: ');
if reference(end) > reference(1)
    overshoot = max(current) - reference(end);
else
    overshoot = min(current) - reference(end);
end
overshoot_percent = abs(overshoot) / abs(reference(end) - reference(1)) * 100;
fprintf('%.4f rad (%.2f%%)\n', overshoot, overshoot_percent);

fprintf('Steady-state error: %.6f rad\n', mean(error(end-100:end)));
fprintf('RMS error: %.6f rad\n', sqrt(mean(error.^2)));

% Save figure
[~, name, ~] = fileparts(filename);
saveas(gcf, [name '_plot.png']);
fprintf('\nFigure saved as: %s_plot.png\n', name);
