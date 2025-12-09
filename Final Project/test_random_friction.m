% Quick test script for random friction simulation
% This demonstrates the walker handling changing friction coefficients

clear; close all; clc;

% Add paths
addpath('./gen');
if ~exist('./gen', 'dir')
    error('Please run starter_code (1).m first to generate functions');
end

fprintf('=== Testing Random Friction Simulation ===\n\n');

%% Random Friction Profile
% Friction changes randomly at each step between 0.3 and 1.2
mu_profile = 'random';  % or use: [0.3, 1.2] for explicit range

fprintf('Friction profile: Random between 0.3 and 1.2\n');

%% Simple Controller
controller = @(t, x) simple_pd_controller(t, x);

%% Initial Conditions
x0 = [0;      % x
      1.0;    % y (hip height)
      -0.2;   % q1
      0.2;    % q2
      0.0;    % q3
      0.5;    % dx
      0.0;    % dy
      0.0;    % dq1
      0.0;    % dq2
      0.0];   % dq3

%% Run Simulation
tspan = [0, 3.0];  % 3 seconds

fprintf('Running simulation...\n');
tic;
[t_sol, x_sol, t_I, mu_history, domain_history] = ...
    simulate_hybrid_walker_variable_mu(x0, tspan, controller, mu_profile);
toc;

%% Display Results
fprintf('\n=== Results ===\n');
fprintf('Number of steps: %d\n', length(t_I));
fprintf('Friction statistics:\n');
fprintf('  Mean: %.3f\n', mean(mu_history));
fprintf('  Std:  %.3f\n', std(mu_history));
fprintf('  Min:  %.3f\n', min(mu_history));
fprintf('  Max:  %.3f\n', max(mu_history));
fprintf('Final position: x = %.3f m\n', x_sol(end,1));

% Count domains
stick_count = sum(strcmp(domain_history, 'stick'));
slip_count = sum(strcmp(domain_history, 'slip'));
fprintf('Domain distribution:\n');
fprintf('  Stick: %d steps (%.1f%%)\n', stick_count, 100*stick_count/length(domain_history));
fprintf('  Slip:  %d steps (%.1f%%)\n', slip_count, 100*slip_count/length(domain_history));

%% Plot Results
figure('Position', [100, 100, 1200, 800]);

% Position
subplot(3,2,1);
plot(t_sol, x_sol(:,1), 'b-', 'LineWidth', 1.5);
hold on;
if ~isempty(t_I) && max(t_I) <= length(x_sol)
    plot(t_sol(t_I), x_sol(t_I,1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    legend('x Position', 'Impacts', 'Location', 'best');
else
    legend('x Position', 'Location', 'best');
end
xlabel('Time (s)');
ylabel('x (m)');
title('Horizontal Position');
grid on;

% Height
subplot(3,2,2);
plot(t_sol, x_sol(:,2), 'g-', 'LineWidth', 1.5);
hold on;
if ~isempty(t_I) && max(t_I) <= length(x_sol)
    plot(t_sol(t_I), x_sol(t_I,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
xlabel('Time (s)');
ylabel('y (m)');
title('Hip Height');
grid on;

% Friction history
subplot(3,2,3);
plot(1:length(mu_history), mu_history, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('Step');
ylabel('\mu');
title('Friction Coefficient History');
grid on;
ylim([0, max(1.5, max(mu_history)*1.1)]);

% Domain history
subplot(3,2,4);
domain_numeric = zeros(size(domain_history));
for i = 1:length(domain_history)
    domain_numeric(i) = strcmp(domain_history{i}, 'stick');
end
stairs(1:length(domain_history), domain_numeric, 'LineWidth', 2);
ylabel('Domain');
xlabel('Step');
title('Domain History (1=Stick, 0=Slip)');
ylim([-0.2, 1.2]);
grid on;

% Joint angles
subplot(3,2,5);
plot(t_sol, x_sol(:,3)*180/pi, 'b-', 'LineWidth', 1.5);
hold on;
plot(t_sol, x_sol(:,4)*180/pi, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Joint Angles');
legend('q1', 'q2', 'Location', 'best');
grid on;

% Velocities
subplot(3,2,6);
plot(t_sol, x_sol(:,6), 'b-', 'LineWidth', 1.5);
hold on;
plot(t_sol, x_sol(:,7), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Hip Velocities');
legend('dx', 'dy', 'Location', 'best');
grid on;

sgtitle('Random Friction Simulation Results', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('\nSimulation complete! Check the plots.\n');

%% Simple PD Controller
function u = simple_pd_controller(t, x)
    % Simple PD controller for testing
    
    % Desired angles
    q1_des = 0.0;
    q2_des = 0.0;
    
    % Current state
    q1 = x(3);
    q2 = x(4);
    dq1 = x(8);
    dq2 = x(9);
    
    % PD gains
    Kp = 50;
    Kd = 10;
    
    % Control
    u1 = -Kp * (q1 - q1_des) - Kd * dq1;
    u2 = -Kp * (q2 - q2_des) - Kd * dq2;
    
    u = [u1; u2];
end

