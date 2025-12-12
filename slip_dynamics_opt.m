function [s_dot] = slip_dynamics_opt(s, u)
    f_val = fs_gen(s);
    g_val = gs_gen(s);
    s_dot = f_val + g_val * u;
end
