function [cineq, ceq] = constraints(z, params)

    MAX_TIME_STEPS = 1000;
    MAX_CONSTRAINTS = MAX_TIME_STEPS * 5;
    tol = 1e-3;

    try
        x0 = z(1:10);
        alpha = z(11:14);
        beta  = z(15:18);

        params1 = params;
        params1.alpha = alpha;
        params1.beta  = beta;

        [t, x_w_cost, x_plus, mode_all] = simulate_single_orbit(z, params1);

        if isempty(t)
            ceq = ones(9, 1) * 1e3;
            cineq = ones(MAX_CONSTRAINTS, 1) * 1e3;
            return;
        end

        x = x_w_cost(:, 1:10);
        N = length(t);
        Ns = min(N, MAX_TIME_STEPS);
        idxs = unique(round(linspace(1, N, Ns)));
        Ns = numel(idxs);

        ceq = x0(2:10) - x_plus(2:10);

        cineq = ones(MAX_CONSTRAINTS, 1) * -1e6;

        for j = 1:Ns
            k = idxs(j);
            xk = x(k, :)';
            mk = mode_all(k);
            u = io_linearization(t(k), xk, params1, mk);

            if mk == "stick"
                Fst = Fst_gen(xk, u);
            else
                Fst = Fst_s_gen(xk, u);
            end
            Fx = Fst(1);
            Fz = Fst(2);
            cone = abs(Fx) - params1.mu * Fz;

            pSt = pSt_gen(xk);
            dpSt = dpSt_gen(xk);

            idx = (j - 1) * 5;
            cineq(idx + 1) = -Fz;
            cineq(idx + 2) = cone;
            cineq(idx + 3) = abs(pSt(2)) - tol;
            cineq(idx + 4) = abs(dpSt(2)) - tol;
            if mk == "stick"
                cineq(idx + 5) = abs(dpSt(1)) - tol;
            else
                cineq(idx + 5) = -1e6;
            end
        end

    catch
        ceq = ones(9, 1) * 1e3;
        cineq = ones(MAX_CONSTRAINTS, 1) * 1e3;
    end
end
