function [t_all, x_all] = simulate_gait(z_star, params)

    Tmax = 20 ;         
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
            break ;
        end


        x_minus = x(end, :)' ;
    
        % Index of event that has fired 
        % 1: Impact Event
        % 2: Continous Transition 
        e = ie(end) ;

        if e == 1
            % Stick or slip accounted for inside func
            x_plus = impact(te(end), x_minus, params) ;

            x0     = x_plus ;         
            t0     = te(end) ;

            % Add logic for slip or stiock dynamics after the impact
            mode = "stick";

        elseif e == 2
           
            x0 = x_minus ;
            t0 = te(end);
            
            %This part still needs the correct logic 
            if mode == "stick"
                mode = "slip";
            else
                mode = "stick";
            end
        end
    end
end
