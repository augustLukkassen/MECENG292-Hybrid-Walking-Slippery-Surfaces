function [t_all, x_all, x_plus] = simulate_single_orbit(z_star, params)

    Tmax = 20 ;         
    t0   = 0 ;
    x0 = [z_star(1:10)] ; % Or just what was given in 7  

    mode = "stick";

    t_all = [] ;
    x_all = [] ;


  
    k = 0 ;
    impact_bool = false ; 

    while impact_bool == false
        k = k + 1;


        switch mode
            case "stick"
                f_dyn = @(t,s) stick_dynamics(t, s, z_star, params) ;
                event = @(t,s) stick_events(t, s, params) ; 
            case "slip"
                f_dyn = @(t,s) slip_dynamics(t, s, z_star, params) ;
                event = @(t,s) slip_events(t, s, params) ; 
        end

        options = odeset('Events', event, 'RelTol',1e-6, 'AbsTol',1e-8) ;

        [t, x, te, ye, ie] = ode45(f_dyn, [t0 Tmax], x0, options) ;

        x_all = [x_all; x];
        t_all = [t_all; t];

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

        if e == 1
            impact_bool = true ; 
            % Stick or slip accounted for inside func
            x_plus = impact(te(end), x_minus, params) ;

            % after impact you usually start in stick mode again

        elseif e == 2
            x0 = x_minus ;
            t0 = te(end);
          
            if mode == "stick"
                mode = "slip";
            else
                lambda = Fst_gen(x_minus, params);
                lambda_x = lambda(1) ; 
                lambda_z = lambda(2) ; 
                mu = params.mu ; 
                % Double check the x vs z
                value = abs(lambda_x) - mu*lambda_z; 
                if value > 0 
                    mode = "slip" ;
                else
                    mode = "stick" ;
                end

            end
        end
    end
end
