function [s_dot] = stick_dynamics(t, s, z, params)
    mode = "stick" ; 
    %s = s(1:10) ; 
    u = io_linearization(t, s, params, mode) ;

    f_val = f_gen(s);
    g_val = g_gen(s);
    s_dot = f_val + g_val * u;

    %dcost = u(1)^2 + u(2)^2 ; 

    %s_dot = [s_dot; dcost] ; 
end
