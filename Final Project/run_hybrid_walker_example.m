% Example script: Run 3-link bipedal walker with variable friction
% This demonstrates how to simulate the walker on hybrid surfaces
% with changing friction coefficients

clear; close all; clc;

% Add paths
addpath('./gen');
if ~exist('./gen', 'dir')
    error('Please run starter_code (1).m first to generate functions');
end

%% Define Friction Profile
% Option 1: Random friction (changes at each step)
% mu_profile = 'random';  % Random between 0.3 and 1.2

% Option 2: Constant friction
% mu_profile = 0.8;

% Option 3: Friction range [min, max] - random in range
% mu_profile = [0.4, 1.0];

% Option 4: Custom function - friction varies with position
mu_profile = @(x, t) 0.5 + 0.3 * sin(0.5 * x(1)) + 0.2 * rand();  % Varies with x position

% Option 5: Step changes in friction (simulates different surface patches)
% mu_values = [0.6, 0.9, 0.4, 1.0, 0.7];  % Different friction zones
% zone_width = 1.0;  % Width of each zone
% mu_profile = @(x, t) mu_values(min(floor(x(1)/zone_width) + 1, length(mu_values)));

fprintf('Using friction profile: variable with position\n');

%% Define Controller
% Walking controller that generates a smooth gait
% The controller should return u = [u1; u2] for the two actuators

% Use walking controller for smooth gait
controller = @(t, x) walking_controller(t, x);

%% Initial Conditions
% State: [x; y; q1; q2; q3; dx; dy; dq1; dq2; dq3]
% Better initial conditions for walking:
% - Stance leg (leg 1) slightly back
% - Swing leg (leg 2) forward and ready to swing
% - Small forward velocity
% - Hip at appropriate height

lL = 1.0;  % Leg length (for initial condition calculation)

x0 = [0;           % x position
      lL*0.85;     % y position (hip height, below leg length so stance foot on ground)
      -0.18;       % q1 (stance leg angle - back, foot on ground)
      0.4;         % q2 (swing leg angle - forward and up)
      0.08;        % q3 (torso angle - forward lean for forward motion)
      0.8;         % dx (forward velocity - higher for walking)
      0.0;         % dy (vertical velocity)
      0.0;         % dq1 (stance leg stationary)
      -0.3;        % dq2 (swing leg moving down/forward)
      0.0];        % dq3

%% Time Span
tspan = [0, 8.0];  % Simulate for 8 seconds (longer for more steps)

%% Run Simulation
fprintf('Starting simulation...\n');
tic;
[t_sol, x_sol, t_I, mu_history, domain_history] = ...
    simulate_hybrid_walker_variable_mu(x0, tspan, controller, mu_profile);
sim_time = toc;
fprintf('Simulation completed in %.2f seconds\n', sim_time);

%% Plot Results
figure(1);
subplot(3,1,1);
plot(t_sol, x_sol(:,1), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('x position (m)');
title('Robot Position');
grid on;
hold on;
if ~isempty(t_I) && max(t_I) <= length(x_sol)
    plot(t_sol(t_I), x_sol(t_I,1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    legend('x Position', 'Impacts', 'Location', 'best');
else
    legend('x Position', 'Location', 'best');
end

subplot(3,1,2);
plot(t_sol, x_sol(:,2), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('y position (m)');
title('Hip Height');
grid on;
hold on;
if ~isempty(t_I) && max(t_I) <= length(x_sol)
    plot(t_sol(t_I), x_sol(t_I,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end

subplot(3,1,3);
if ~isempty(mu_history)
    plot(1:length(mu_history), mu_history, 'r-', 'LineWidth', 1.5);
    xlabel('Step');
    ylabel('Friction Coefficient \mu');
    title('Friction History');
    grid on;
    ylim([0, max(1.5, max(mu_history)*1.1)]);
else
    text(0.5, 0.5, 'No friction data', 'HorizontalAlignment', 'center');
    xlabel('Step');
    ylabel('Friction Coefficient \mu');
    title('Friction History');
end

%% Plot Domain History
if ~isempty(domain_history)
    figure(2);
    domain_numeric = zeros(size(domain_history));
    for i = 1:length(domain_history)
        if strcmp(domain_history{i}, 'stick')
            domain_numeric(i) = 1;
        else
            domain_numeric(i) = 0;
        end
    end
    stairs(1:length(domain_history), domain_numeric, 'LineWidth', 2);
    ylabel('Domain');
    xlabel('Step');
    title('Domain History (1=Stick, 0=Slip)');
    ylim([-0.2, 1.2]);
    grid on;
end

%% Animate
% Set animate_flag to true to show animation, false to skip
animate_flag = true;

if animate_flag && exist('animate_three_link_walker.m', 'file')
    fprintf('\nStarting animation...\n');
    fprintf('Close the animation window or press Ctrl+C to stop early.\n');
    try
        if ~isempty(t_I) && length(t_I) > 0
            % Animate with step information
            animate_three_link_walker(t_sol, x_sol, t_I, 1.5);  % 1.5x speed for smooth viewing
        else
            % No impacts recorded, animate entire trajectory
            animate_three_link_walker(t_sol, x_sol, [], 1.5);  % 1.5x speed
        end
        fprintf('Animation complete.\n');
    catch ME
        if strcmp(ME.identifier, 'MATLAB:handle:InvalidHandle') || ...
           contains(ME.message, 'Figure')
            fprintf('Animation window closed by user.\n');
        else
            warning('Animation failed: %s', ME.message);
            fprintf('Skipping animation.\n');
        end
    end
elseif animate_flag
    fprintf('Animation function not found. Skipping animation.\n');
else
    fprintf('Animation skipped (set animate_flag = true to enable).\n');
end

fprintf('\nSimulation Summary:\n');
fprintf('  Total steps: %d\n', length(t_I));
if ~isempty(mu_history)
    fprintf('  Average friction: %.3f\n', mean(mu_history));
    fprintf('  Friction range: [%.3f, %.3f]\n', min(mu_history), max(mu_history));
else
    fprintf('  No friction data recorded\n');
end
if ~isempty(x_sol)
    fprintf('  Final position: x = %.3f m\n', x_sol(end,1));
else
    fprintf('  No position data\n');
end

%% Walking Controller Function
function u = walking_controller(t, x)
    % Walking controller that generates a smooth bipedal gait
    % This controller ensures the swing leg comes down to create impacts
    
    % Add path to generated functions if needed
    if exist('./gen', 'dir')
        addpath('./gen');
    end
    
    % Current state
    q1 = x(3);   % Stance leg angle (leg 1)
    q2 = x(4);   % Swing leg angle (leg 2)
    q3 = x(5);   % Torso angle
    dq1 = x(8);
    dq2 = x(9);
    dq3 = x(10);
    
    % Get swing foot height to determine if we need to bring it down
    z_sw = 1.0;  % Default
    try
        if exist('pSw_gen', 'file') == 2
            pSw = pSw_gen(x);
            z_sw = pSw(2);  % Swing foot height
        end
    catch
        % If function not available, estimate from state
        % Approximate: swing foot height depends on q2 and hip height
        y_hip = x(2);
        q2_abs = q2 + q3;
        z_sw = y_hip - 1.0 * sin(q2_abs - pi/2);  % Rough estimate
    end
    
    % Walking parameters
    omega = 1.5;  % Walking frequency (rad/s) - faster for more steps
    phase = omega * t;
    
    % Desired trajectories for walking gait
    % Stance leg: stays relatively straight and slightly back
    q1_des = -0.15;  % Keep stance leg back (constant)
    
    % Swing leg: MUST come down to ground
    % Use a trajectory that actively brings the leg down
    % The key is to increase q2 to bring the leg forward and down
    
    % Base swing trajectory - forward motion
    q2_base = 0.3 * sin(phase) + 0.2;
    
    % Active downward motion when swing foot is high
    if z_sw > 0.15  % If swing foot is above 15cm, bring it down
        % Increase q2 to bring leg forward and down
        q2_des = q2_base + 0.3 * (z_sw / 1.0);  % Proportional to height
    elseif z_sw > 0.05  % Getting close to ground
        % Continue bringing it down but slower
        q2_des = q2_base + 0.15;
    else
        % Near ground, prepare for next swing
        q2_des = q2_base;
    end
    
    % Ensure q2_des is reasonable (not too extreme)
    q2_des = max(-0.5, min(0.8, q2_des));
    
    % Torso: slight forward lean for forward motion
    q3_des = 0.08;  % Forward lean
    
    % Desired velocities
    dq1_des = 0.0;  % Stance leg relatively stationary
    dq2_des = 0.3 * omega * cos(phase);  % Base swing leg velocity
    
    % Add downward velocity when swing foot is high
    if z_sw > 0.1
        dq2_des = dq2_des - 0.8;  % Strong downward motion
    end
    
    dq3_des = 0.0;  % Torso relatively stable
    
    % PD control gains - higher for swing leg to ensure it moves
    Kp1 = 60;   % Stance leg position gain
    Kd1 = 12;   % Stance leg damping
    Kp2 = 120;  % Swing leg position gain (higher to ensure motion)
    Kd2 = 25;   % Swing leg damping
    
    % Control inputs
    u1 = -Kp1 * (q1 - q1_des) - Kd1 * (dq1 - dq1_des);
    u2 = -Kp2 * (q2 - q2_des) - Kd2 * (dq2 - dq2_des);
    
    u = [u1; u2];
end

%% Simple Controller Function (kept for reference)
function u = simple_controller(t, x)
    % Simple PD controller for demonstration
    % You should replace this with your actual controller
    
    % Desired angles (can be trajectory-based)
    q1_des = 0.0;
    q2_des = 0.0;
    
    % Current angles
    q1 = x(3);
    q2 = x(4);
    dq1 = x(8);
    dq2 = x(9);
    
    % PD gains
    Kp = 50;
    Kd = 10;
    
    % Control inputs
    u1 = -Kp * (q1 - q1_des) - Kd * dq1;
    u2 = -Kp * (q2 - q2_des) - Kd * dq2;
    
    u = [u1; u2];
end

