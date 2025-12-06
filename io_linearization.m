function u = io_linearization(t, s, z, mode)
    a = 0.9;
    epsilon = 0.1;

    a0 = z(11); a1 = z(12); a2 = z(13); a3 = z(14);
    b0 = z(15); b1 = z(16); b2 = z(17); b3 = z(18);
    
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