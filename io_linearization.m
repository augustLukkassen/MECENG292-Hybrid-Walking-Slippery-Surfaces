function u = io_linearization(t, s, params, mode)
    a = 0.9;
    epsilon = 0.1;

    % Extract parameters from params struct
    alpha = params.alpha;
    beta = params.beta;

    a0 = alpha(1); a1 = alpha(2); a2 = alpha(3); a3 = alpha(4);
    b0 = beta(1); b1 = beta(2); b2 = beta(3); b3 = beta(4);
    
    if mode == "stick"
        [y, Lfy, Lf2y, LgLfy] = lie_derivatives_gen(s, ...
            a0, a1, a2, a3, ...
            b0, b1, b2, b3) ;
    else % ie mode is slip 
        [y, Lfy, Lf2y, LgLfy] = lie_derivatives_gen_s(s, ...
            a0, a1, a2, a3, ...
            b0, b1, b2, b3) ;
    end
   
    
    % HW 7 Controller 
    dy1 = Lfy(1);
    dy2 = Lfy(2);

    phi_a1 = y(1) + 1/(2-a) * sign(epsilon*dy1)*abs(epsilon*dy1)^(2-a);
    phi_a2 = y(2) + 1/(2-a) * sign(epsilon*dy2)*abs(epsilon*dy2)^(2-a);

    psi_a1 = -sign(epsilon*dy1)*abs(epsilon*dy1)^a - sign(phi_a1)*abs(phi_a1)^(a/(2-a));
    psi_a2 = -sign(epsilon*dy2)*abs(epsilon*dy2)^a - sign(phi_a2)*abs(phi_a2)^(a/(2-a));

    v = (1/epsilon^2) * [psi_a1; psi_a2];

    % Normal PD Controller 
    kp = 100;
    kd = 20;
    y_dot = Lfy;
    %v = -kp * y - kd * y_dot;
    
    % This part is the same
    u = LgLfy \ (-Lf2y + v);
end