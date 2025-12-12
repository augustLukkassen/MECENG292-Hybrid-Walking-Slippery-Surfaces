function cost = periodic_orbit_obj(z, params)
    try
        alpha = z(11:14);
        beta  = z(15:18);
        params1 = params;
        params1.alpha = alpha;
        params1.beta  = beta;

        [t, x_w_cost, ~] = simulate_single_orbit(z, params1);
        if isempty(t)
            cost = 1e9;
            return;
        end

        integral_cost = x_w_cost(end, 11);
        distance_traveled = x_w_cost(end, 1) - x_w_cost(1, 1);

        if abs(distance_traveled) < 1e-6
            cost = 1e9;
        else
            cost = integral_cost / distance_traveled;
        end
    catch
        cost = 1e9;
    end
end


