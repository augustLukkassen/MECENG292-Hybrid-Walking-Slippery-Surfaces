function [t_all, x_all] = simulate_gait(z_star, params)

    Tmax = 1.5 ;         
    t0   = 0 ;
    x0 = [z_star(1:10)] ; % Or just what was given in 7  

    mode = "stick";

    t_all = [] ;
    x_all = [] ;


    maxEvents = 100;
    k = 0 ;

    while t0 < Tmax && k < maxEvents
        k = k + 1;


        switch mode
            case "stick"
                f_dyn = @(t,s) stick_dynamics(t, s, z_star, params) ;
                event = @(t,s) stick_events(t, s, z_star, params) ; 
            case "slip"
                f_dyn = @(t,s) slip_dynamics(t, s, z_star, params) ;
                event = @(t,s) slip_events(t, s, z_star, params) ; 
            end

        % Integrator settings:
        % - Start with ode45 (fast for non-stiff)
        % - If it hits tolerance/step-size failure, fall back to ode15s
        relTol = 1e-4;
        absTol = 1e-6;
        options = odeset('Events', event, 'RelTol', relTol, 'AbsTol', absTol, 'MaxStep', 1e-2) ;

        lastwarn('');
        [t, x, te, ye, ie] = ode45(f_dyn, [t0 Tmax], x0, options) ;
        [wmsg, ~] = lastwarn;
        if ~isempty(wmsg) && contains(wmsg, 'Unable to meet integration tolerances')
            % Try a stiff solver with slightly looser tolerances
            options15 = odeset(options, 'RelTol', 1e-3, 'AbsTol', 1e-5, 'MaxStep', 5e-3);
            lastwarn('');
            [t, x, te, ye, ie] = ode15s(f_dyn, [t0 Tmax], x0, options15) ;
        end

        x_all = [x_all; x];
        t_all = [t_all; t];

        % No events means that ode exits from timeout 
        if isempty(te)
            break ;
        end


        x_minus = x(end, :)' ;
    
        % Index of event that has fired 
        % 1: Impact Event
        % 2: Continous Transition 
        e = ie(end) ;

        if e == 1
            % Stick or slip accounted for inside func
            
            [x_plus, next_mode] = impact(x_minus, params) ;

            x0     = x_plus ;         
            t0     = te(end) ;

            % Add logic for slip or stiock dynamics after the impact
            mode = next_mode;

        elseif e == 2
           
            x0 = x_minus ;
            t0 = te(end);
            
            %This part still needs the correct logic 
            if mode == "stick"
                mode = "slip";
            else
                % Slip -> Stick transition check
                u_stick = io_linearization(t0, x_minus, params, "stick");
                lambda = Fst_gen(x_minus, u_stick);
                
                lambda_x = lambda(1) ; 
                lambda_z = lambda(2) ; 
                mu = params.mu ; 
                
                if (abs(lambda_x) - mu*lambda_z) > 0
                    mode = "slip";
                else
                    mode = "stick";
                end
            end
        end
    end
end
