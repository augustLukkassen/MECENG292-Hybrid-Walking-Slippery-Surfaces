function [t_all, x_all, x_plus, mode_all] = simulate_single_orbit(z_star, params)

    if isfield(params, 'Tmax')
        Tmax = params.Tmax;
    else
        Tmax = 5;
    end
    t0   = 0 ;
    x0 = [z_star(1:10); 0] ;

    mode = "stick";

    t_all = [] ;
    x_all = [] ;
    mode_all = [] ;


  
    k = 0 ;
    impact_bool = false ; 
    if isfield(params, 'max_segments')
        max_segments = params.max_segments;
    else
        max_segments = 50;
    end
    if isfield(params, 'debug')
        debug = params.debug;
    else
        debug = false;
    end

    while impact_bool == false
        k = k + 1;
        if k > max_segments
            t_all = [];
            x_all = [];
            x_plus = [];
            mode_all = [];
            return;
        end


        switch mode
            case "stick"
                f_dyn = @(t,s) stick_dynamics(t, s, z_star, params) ;
                event = @(t,s) stick_events(t, s, z_star, params) ; 
            case "slip"
                f_dyn = @(t,s) slip_dynamics(t, s, z_star, params) ;
                event = @(t,s) slip_events(t, s, z_star, params) ; 
            end

        options = odeset('Events', event, 'RelTol',1e-6, 'AbsTol',1e-8) ;

        [t, x, te, ye, ie] = ode45(f_dyn, [t0 Tmax], x0, options) ;

        x_all = [x_all; x];
        t_all = [t_all; t];
        mode_all = [mode_all; repmat(mode, length(t), 1)];

        % No events means that ode exits from timeout 
        if isempty(te)
            disp("No impact before timout")
            break ;
        end


        x_minus = x(end, :)' ;
    
        % Index of event that has fired 
        % 1: Impact Event
        % 2: Continous Transition 
        e = ie(end) ;

        if debug
            disp([char(mode) '  seg=' num2str(k) '  e=' num2str(e) '  t=' num2str(te(end))])
        end

        if e == 1
            impact_bool = true ; 
            % Stick or slip accounted for inside func
            [x_plus10, next_mode] = impact(x_minus(1:10), params) ;
            x_plus = [x_plus10; x_minus(11)];
            % after impact you usually start in stick mode again
        elseif e == 2
            x0 = x_minus ;
            t0 = te(end) + 1e-6;
          
            if mode == "stick"
                mode = "slip";
            else
                % We are in Slip, event detected zero velocity.
                % Check if we can Stick (forces within friction cone).
                
                % 1. Compute Control assuming Stick
                u_stick = io_linearization(t0, x_minus(1:10), params, "stick");
                
                % 2. Compute Stick Forces
                lambda = Fst_gen(x_minus(1:10), u_stick);
                lambda_x = lambda(1) ; 
                lambda_z = lambda(2) ; 
                mu = params.mu ; 
                
                % 3. Check Friction Cone
                value = abs(lambda_x) - mu*lambda_z; 
                if value > 0 
                    % Forces violate friction cone -> Continue Slipping
                    mode = "slip" ;
                else
                    % Forces ok -> Stick
                    mode = "stick" ;
                end
            end
        end

    end
end
