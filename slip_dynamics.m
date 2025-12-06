function [s_dot] = slip_dynamics(t, s, z)
    mode = "slip" ; 

    %s = s(1:10) ; 
    u = io_linearization(t, s, z, mode) ;

    f_val = fs_gen(s);
    g_val = gs_gen(s);
    s_dot = f_val + g_val * u;

    %dcost = u(1)^2 + u(2)^2 ; 

    %s_dot = [s_dot; dcost] ; 
end