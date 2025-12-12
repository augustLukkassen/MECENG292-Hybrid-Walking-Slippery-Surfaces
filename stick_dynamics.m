function s_dot = stick_dynamics(t, s, z, params)
    mode = "stick";

    if numel(s) >= 10
        s10 = s(1:10);
    else
        s10 = s;
    end

    u = io_linearization(t, s10, params, mode);

    f_val = f_gen(s10);
    g_val = g_gen(s10);
    s_dot10 = f_val + g_val * u;

    if numel(s) == 11
        dcost = u(1)^2 + u(2)^2;
        s_dot = [s_dot10; dcost];
    else
        s_dot = s_dot10;
    end
end
