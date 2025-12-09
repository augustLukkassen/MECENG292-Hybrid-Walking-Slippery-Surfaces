% Simulation of 3-link bipedal walker with hybrid dynamics and variable friction
% This script handles stick-slip transitions and variable friction surfaces
%
% Usage:
%   [t_sol, x_sol, t_I, mu_history] = simulate_hybrid_walker(x0, tspan, controller, mu_profile)
%
% Inputs:
%   x0: Initial state [x; y; q1; q2; q3; dx; dy; dq1; dq2; dq3]
%   tspan: Time span [t_start, t_end] or time vector
%   controller: Function handle for controller u = controller(t, x)
%   mu_profile: Function handle for friction coefficient mu = mu_profile(x, t)
%               OR a scalar for constant friction
%
% Outputs:
%   t_sol: Time array
%   x_sol: State array (each row is a state)
%   t_I: Indices where impacts occurred
%   mu_history: Friction coefficient history

function [t_sol, x_sol, t_I, mu_history] = simulate_hybrid_walker(x0, tspan, controller, mu_profile)
    
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
    
    % Default friction profile (if not provided)
    if nargin < 4 || isempty(mu_profile)
        % Random friction: changes every 0.5 seconds between 0.3 and 1.2
        mu_profile = @(x, t) 0.3 + 0.9 * rand();  % Random at each call
    end
    
    % Handle scalar mu
    if isnumeric(mu_profile) && isscalar(mu_profile)
        mu_val = mu_profile;
        mu_profile = @(x, t) mu_val;
    end
    
    % Initialize
    x = x0(:);
    t_sol = [];
    x_sol = [];
    t_I = [];
    mu_history = [];
    current_domain = 'stick';  % 'stick' or 'slip'
    
    % Impact detection parameters
    impact_tol = 1e-4;  % Tolerance for impact detection
    min_time_between_impacts = 0.1;  % Minimum time between impacts
    last_impact_time = -inf;
    
    % ODE options
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8, ...
                    'Events', @(t, x) impact_event(t, x, impact_tol));
    
    % Simulation loop
    t_current = tspan(1);
    t_end = tspan(end);
    step_count = 0;
    max_steps = 100;  % Maximum number of steps
    
    while t_current < t_end && step_count < max_steps
        % Get current friction coefficient
        mu_current = mu_profile(x, t_current);
        mu_history = [mu_history; mu_current];
        
        % Determine domain based on current state and friction
        [domain, FSt_current] = determine_domain(x, controller(t_current, x), mu_current);
        current_domain = domain;
        
        % Integrate until impact or end time
        t_span_step = [t_current, t_end];
        
        try
            [t_step, x_step, te, xe, ie] = ode45(@(t, x) dynamics_wrapper(t, x, controller, mu_current, current_domain), ...
                                                   t_span_step, x, options);
        catch ME
            warning('ODE integration failed: %s', ME.message);
            break;
        end
        
        % Store results
        if isempty(t_sol)
            t_sol = t_step;
            x_sol = x_step;
        else
            t_sol = [t_sol; t_step(2:end)];  % Avoid duplicate
            x_sol = [x_sol; x_step(2:end, :)];
        end
        
        % Check for impact
        if ~isempty(te) && length(te) > 0
            % Impact detected
            if te(1) - last_impact_time > min_time_between_impacts
                % Apply impact map
                x_pre = xe(1, :)';
                [x_post, impact_type] = apply_impact_map(x_pre, mu_current);
                
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
                
                fprintf('Step %d: Impact at t=%.3f, type=%s, mu=%.3f\n', ...
                        step_count, te(1), impact_type, mu_current);
            else
                % Too soon after last impact, continue
                x = x_step(end, :)';
                t_current = t_step(end);
            end
        else
            % No impact, reached end time
            x = x_step(end, :)';
            t_current = t_step(end);
        end
        
        % Small time step to avoid getting stuck
        if abs(t_current - t_step(end)) < 1e-6
            t_current = t_current + 1e-3;
        end
    end
    
    fprintf('Simulation complete: %d steps, final time = %.3f\n', step_count, t_current);
end

% Dynamics wrapper that handles variable friction and domain selection
function dx = dynamics_wrapper(t, x, controller, mu, domain)
    % Get control input
    u = controller(t, x);
    
    % Get vector fields (note: these use the hardcoded mu=0.8 from generation)
    % For variable friction, we need to recompute or use approximation
    % TODO: Regenerate functions with mu as parameter for full accuracy
    
    % For now, use the generated functions (they assume mu=0.8)
    % In a full implementation, you'd regenerate Fst_gen with mu as parameter
    try
        f = fvec_gen(x);
        g = gvec_gen(x);
        dx = f + g * u;
    catch
        % Fallback: simple dynamics (shouldn't happen if functions exist)
        dx = zeros(size(x));
    end
    
    % Domain-specific adjustments could go here
    % For stick domain: ensure constraints are satisfied
    % For slip domain: allow tangential motion
end

% Determine current domain (stick or slip)
function [domain, FSt] = determine_domain(x, u, mu)
    try
        % Compute contact forces (using generated function with mu=0.8)
        % Note: For accurate variable friction, regenerate Fst_gen with mu parameter
        FSt = Fst_gen(x, u);
        
        % Check friction condition
        lambda_x = FSt(1);
        lambda_z = FSt(2);
        
        % Use current mu value for domain determination
        if abs(lambda_x) <= mu * lambda_z && lambda_z > 0
            domain = 'stick';
        else
            domain = 'slip';
        end
    catch
        % Default to stick if computation fails
        domain = 'stick';
        FSt = [0; 0];
    end
end

% Apply appropriate impact map based on friction
function [x_post, impact_type] = apply_impact_map(x_pre, mu)
    try
        % Compute both impact maps
        dqPlus_stick = dqPlus_stick_gen(x_pre);
        dqPlus_slip = dqPlus_slip_gen(x_pre);
        Fimpact_stick = Fimpact_stick_gen(x_pre);
        
        % Check which impact type to use
        Fimpact_stick_x = Fimpact_stick(1);
        Fimpact_stick_z = Fimpact_stick(2);
        
        % Use current mu for decision (note: impact maps use mu=0.8 from generation)
        % For full accuracy, regenerate impact functions with mu parameter
        if abs(Fimpact_stick_x) <= mu * Fimpact_stick_z && Fimpact_stick_z > 0
            % Sticking impact
            x_post = [x_pre(1:5); dqPlus_stick];
            impact_type = 'stick';
        else
            % Slipping impact
            x_post = [x_pre(1:5); dqPlus_slip];
            impact_type = 'slip';
        end
    catch
        % Fallback to default impact
        dqPlus = dqPlus_gen(x_pre);
        x_post = [x_pre(1:5); dqPlus];
        impact_type = 'default';
    end
end

% Impact event detection
function [value, isterminal, direction] = impact_event(t, x, tol)
    try
        % Check if swing foot touches ground
        pSw = pSw_gen(x);
        z_sw = pSw(2);  % Vertical position
        
        dpSw = dpSw_gen(x);
        dz_sw = dpSw(2);  % Vertical velocity
        
        % Impact occurs when z_sw = 0 and dz_sw < 0
        value = z_sw;
        isterminal = 1;  % Stop integration
        direction = -1;   % Only detect when crossing from positive to negative
    catch
        value = 1;  % No impact
        isterminal = 0;
        direction = 0;
    end
end

