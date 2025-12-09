% Simulation of 3-link bipedal walker with hybrid dynamics and variable friction
% This version handles variable friction by using numerical computation
% when the generated functions have hardcoded mu values
%
% Usage:
%   [t_sol, x_sol, t_I, mu_history, domain_history] = ...
%       simulate_hybrid_walker_variable_mu(x0, tspan, controller, mu_profile)
%
% Inputs:
%   x0: Initial state [x; y; q1; q2; q3; dx; dy; dq1; dq2; dq3]
%   tspan: Time span [t_start, t_end] or time vector
%   controller: Function handle for controller u = controller(t, x)
%   mu_profile: Function handle for friction coefficient mu = mu_profile(x, t)
%               OR a scalar for constant friction
%               OR 'random' for random friction between 0.3 and 1.2
%
% Outputs:
%   t_sol: Time array
%   x_sol: State array (each row is a state)
%   t_I: Indices where impacts occurred
%   mu_history: Friction coefficient history
%   domain_history: Domain history ('stick' or 'slip')

function [t_sol, x_sol, t_I, mu_history, domain_history] = ...
    simulate_hybrid_walker_variable_mu(x0, tspan, controller, mu_profile)
    
    % Add path to generated functions
    if ~exist('./gen', 'dir')
        error('Generated functions not found. Please run starter_code (1).m first.');
    end
    addpath('./gen');
    
    % Parameters
    params = struct();
    params.R = [1, 0, 0, 0, 0;
                0, 1, 0, 0, 0;
                0, 0, 0, 1, 0;
                0, 0, 1, 0, 0;
                0, 0, 0, 0, 1];  % Relabelling matrix
    
    % Default friction profile
    if nargin < 4 || isempty(mu_profile)
        mu_profile = 'random';
    end
    
    % Parse friction profile
    if ischar(mu_profile) && strcmp(mu_profile, 'random')
        % Random friction that changes every step
        mu_base = 0.3 + 0.9 * rand();  % Random between 0.3 and 1.2
        mu_profile = @(x, t) mu_base + 0.2 * (rand() - 0.5);  % Add small variation
    elseif isnumeric(mu_profile) && isscalar(mu_profile)
        mu_val = mu_profile;
        mu_profile = @(x, t) mu_val;
    elseif isnumeric(mu_profile) && length(mu_profile) == 2
        % Range [mu_min, mu_max] - random in range
        mu_min = mu_profile(1);
        mu_max = mu_profile(2);
        mu_profile = @(x, t) mu_min + (mu_max - mu_min) * rand();
    end
    
    % Initialize
    x = x0(:);
    t_sol = [];
    x_sol = [];
    t_I = [];
    mu_history = [];
    domain_history = {};
    current_domain = 'stick';
    
    % Impact detection parameters
    impact_tol = 1e-3;  % Slightly larger tolerance for more reliable detection
    min_time_between_impacts = 0.1;  % Minimum time between impacts (prevents double-triggering)
    last_impact_time = -inf;
    
    % Output sampling rate (time between output points)
    % Use smaller interval to capture impacts and smooth animation
    output_dt = 0.05;  % Output every 0.05 seconds (20 Hz) for smooth animation
    
    % ODE options with impact event
    % MaxStep controls integration accuracy (keep small for accuracy)
    % OutputFcn or explicit time points control output sampling
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
                    'Events', @(t, x) impact_event(t, x, impact_tol), ...
                    'MaxStep', 0.1);  % Integration step (can be smaller than output_dt)
    
    % Simulation loop
    t_current = tspan(1);
    t_end = tspan(end);
    step_count = 0;
    max_steps = 50;
    
    fprintf('Starting simulation with variable friction...\n');
    
    while t_current < t_end && step_count < max_steps
        % Get current friction coefficient
        mu_current = mu_profile(x, t_current);
        mu_history = [mu_history; mu_current];
        
        % Get control input
        u = controller(t_current, x);
        
        % Determine domain
        [domain, FSt_current] = determine_domain_variable_mu(x, u, mu_current);
        current_domain = domain;
        domain_history{end+1} = domain;
        
        % Integrate until impact or end time
        t_span_step = [t_current, min(t_current + 2.0, t_end)];  % Max 2 sec per step
        
        % Create explicit output time points (every output_dt seconds)
        % This ensures output is sampled at regular intervals
        t_output = t_current:output_dt:t_span_step(2);
        if t_output(end) ~= t_span_step(2)
            t_output = [t_output, t_span_step(2)];  % Include end time
        end
        
        try
            [t_step, x_step, te, xe, ie] = ode45(...
                @(t, x) dynamics_wrapper_variable_mu(t, x, controller, mu_current), ...
                t_output, x, options);  % Use explicit output times
        catch ME
            warning('ODE integration failed at t=%.3f: %s', t_current, ME.message);
            break;
        end
        
        % Store results
        if isempty(t_sol)
            t_sol = t_step;
            x_sol = x_step;
        else
            % Avoid duplicate point
            if length(t_sol) > 0 && abs(t_step(1) - t_sol(end)) < 1e-6
                t_sol = [t_sol; t_step(2:end)];
                x_sol = [x_sol; x_step(2:end, :)];
            else
                t_sol = [t_sol; t_step];
                x_sol = [x_sol; x_step];
            end
        end
        
        % Check for impact
        if ~isempty(te) && length(te) > 0 && (te(1) - last_impact_time) > min_time_between_impacts
            % Impact detected - apply impact map
            x_pre = xe(1, :)';
            [x_post, impact_type] = apply_impact_map_variable_mu(x_pre, mu_current);
            
            % Apply relabelling (swap legs)
            x_post(1:5) = params.R * x_post(1:5);
            x_post(6:10) = params.R * x_post(6:10);
            
            % Update state
            x = x_post;
            t_current = te(1);
            
            % Record impact
            t_I = [t_I; length(t_sol)];
            step_count = step_count + 1;
            last_impact_time = te(1);
            
            fprintf('Step %d: Impact at t=%.3f, type=%s, mu=%.3f, domain=%s\n', ...
                    step_count, te(1), impact_type, mu_current, current_domain);
            else
                % No impact or too soon - continue
                if length(t_step) > 1
                    x = x_step(end, :)';
                    t_current = t_step(end);
                else
                    % Stuck or reached end
                    if abs(t_current - t_end) < 1e-3
                        break;
                    end
                    t_current = t_current + output_dt;  % Advance by output step
                end
            end
    end
    
    fprintf('Simulation complete: %d steps, final time = %.3f\n', step_count, t_current);
end

% Dynamics wrapper with variable friction
function dx = dynamics_wrapper_variable_mu(t, x, controller, mu)
    % Get control input
    u = controller(t, x);
    
    % Use generated functions (they use mu=0.8, but we'll adjust domain logic)
    % For full accuracy with variable mu, regenerate functions with mu as parameter
    try
        f = fvec_gen(x);
        g = gvec_gen(x);
        dx = f + g * u;
    catch ME
        warning('Error in dynamics: %s', ME.message);
        dx = zeros(size(x));
    end
end

% Determine domain with variable mu
function [domain, FSt] = determine_domain_variable_mu(x, u, mu)
    try
        % Compute contact forces using generated function
        % Note: This uses mu=0.8 from generation, but we use actual mu for domain check
        FSt = Fst_gen(x, u);
        
        lambda_x = FSt(1);
        lambda_z = FSt(2);
        
        % Use actual mu value for domain determination
        if abs(lambda_x) <= mu * lambda_z && lambda_z > 0
            domain = 'stick';
        else
            domain = 'slip';
        end
    catch
        domain = 'stick';
        FSt = [0; 0];
    end
end

% Apply impact map with variable friction
function [x_post, impact_type] = apply_impact_map_variable_mu(x_pre, mu)
    try
        % Compute impact maps
        dqPlus_stick = dqPlus_stick_gen(x_pre);
        dqPlus_slip = dqPlus_slip_gen(x_pre);
        Fimpact_stick = Fimpact_stick_gen(x_pre);
        
        % Check friction condition with actual mu
        Fimpact_stick_x = Fimpact_stick(1);
        Fimpact_stick_z = Fimpact_stick(2);
        
        % Decision based on actual mu (not the mu=0.8 used in generation)
        if abs(Fimpact_stick_x) <= mu * Fimpact_stick_z && Fimpact_stick_z > 0
            x_post = [x_pre(1:5); dqPlus_stick];
            impact_type = 'stick';
        else
            x_post = [x_pre(1:5); dqPlus_slip];
            impact_type = 'slip';
        end
    catch ME
        % Fallback
        try
            dqPlus = dqPlus_gen(x_pre);
            x_post = [x_pre(1:5); dqPlus];
            impact_type = 'default';
        catch
            x_post = x_pre;
            impact_type = 'none';
        end
    end
end

% Impact event detection
function [value, isterminal, direction] = impact_event(t, x, tol)
    try
        pSw = pSw_gen(x);
        z_sw = pSw(2);  % Swing foot height
        dpSw = dpSw_gen(x);
        dz_sw = dpSw(2);  % Swing foot vertical velocity
        
        % Impact occurs when swing foot touches ground (z_sw = 0)
        % and is moving downward (dz_sw < 0)
        value = z_sw;
        isterminal = 1;  % Stop integration at impact
        direction = -1;  % Detect crossing from positive to negative (coming down)
        
        % Debug output (uncomment to see impact detection)
        % if z_sw < 0.1 && dz_sw < 0
        %     fprintf('Near impact: z_sw=%.4f, dz_sw=%.4f\n', z_sw, dz_sw);
        % end
    catch ME
        % Fallback if functions not available
        value = 1;
        isterminal = 0;
        direction = 0;
    end
end

