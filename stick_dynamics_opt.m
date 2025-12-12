function [s_dot] = stick_dynamics_opt(s, u)
    f_val = f_gen(s);
    g_val = g_gen(s);
    s_dot = f_val + g_val * u;
end
