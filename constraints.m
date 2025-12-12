function [c, ceq] = constraints(z, N, nx, nu, nF, na, nb, dt, params)

    idx_X_end     = nx * N;
    idx_U_end     = idx_X_end  + nu * N;
    %idx_F_end     = idx_U_end  + nF * N;
    idx_alpha_end = idx_U_end  + na;
    idx_beta_end  = idx_alpha_end + nb;

    X_vec     = z(1                : idx_X_end);
    U_vec     = z(idx_X_end + 1    : idx_U_end);
    %F_vec     = z(idx_U_end + 1    : idx_F_end);
    Alpha_vec = z(idx_U_end + 1    : idx_alpha_end);
    Beta_vec  = z(idx_alpha_end + 1 : idx_beta_end);

    X = reshape(X_vec, nx, N);
    U = reshape(U_vec, nu, N);
    %F = reshape(F_vec, nF, N);

    alpha = Alpha_vec(:);
    beta  = Beta_vec(:);

    a0 = alpha(1); a1 = alpha(2); a2 = alpha(3); a3 = alpha(4) ;
    b0 = beta(1); b1 = beta(2); b2 = beta(3); b3 = beta(4) ;

    % Hardcoded Transition Times 
    N1 = params.N1;
    N2 = params.N2;

    h = 2 * dt;

    ceq = [];
    c   = [];

    minFz = inf; maxFriction = -inf;
    slipCount = 0; stickCount = 0;

    for i = 2:2:(N-1)
        im1 = i - 1;
        ip1 = i + 1;

        x_im1 = X(:, im1);   u_im1 = U(:, im1);   %f_im1 = F(:, im1);
        x_i   = X(:, i);     u_i   = U(:, i);     %f_i   = F(:, i);
        x_ip1 = X(:, ip1);   u_ip1 = U(:, ip1);   %f_ip1 = F(:, ip1);

        if i <= (N1 - 1) || i >= (N2 + 1)
            % Slip Dynamics 
            dyn_fun = @slip_dynamics_opt;
            [y, Lfy, Lf2y, LgLfy] = lie_derivatives_gen_s(x_i, ...
                a0,a1,a2,a3, b0,b1,b2,b3);
            FSt = Fst_s_gen(x_i, u_i);  
            mode = "slip";
            slipCount = slipCount + 1;
        else
            % Stick Dynamics 
            dyn_fun = @stick_dynamics_opt;
            [y, Lfy, Lf2y, LgLfy] = lie_derivatives_gen(x_i, ...
                a0,a1,a2,a3, b0,b1,b2,b3);
            FSt = Fst_gen(x_i, u_i);  
            stickCount = stickCount + 1;
            mode = "stick";
        end

        dx_im1 = dyn_fun(x_im1, u_im1);
        dx_i   = dyn_fun(x_i,   u_i);
        dx_ip1 = dyn_fun(x_ip1, u_ip1);

        defect1 = x_i - 0.5 * (x_im1 + x_ip1) - (h/8) * (dx_im1 - dx_ip1);
        defect2 = x_ip1 - x_im1 - (h/6) * (dx_im1 + 4*dx_i + dx_ip1);

        ceq = [ceq;
               defect1;
               defect2];
        
        
        Fx = FSt(1);
        Fz = FSt(2);
        mu = params.mu;

        minFz = min(minFz, Fz);
        if Fz ~= 0
            maxFriction = max(maxFriction, abs(Fx) / Fz);
        end

        c = [c;
             abs(Fx) - mu * Fz;
            -Fz];

        % Enforce that decision input matches the simulation controller
        % (keeps optimization/controller consistent)
        t_i = (i-1) * dt;
        params1 = params;
        params1.alpha = alpha;
        params1.beta  = beta;
        ceq = [ceq;
               u_i - io_linearization(t_i, x_i, params1, mode)] ;

    end

    x_start = X(:, 1);
    x_end   = X(:, N);

    % Periodicity 
    %mode = "stick" ;
    [x_plus, ~] = impact(x_end, params) ; 
    % Temporarily scale periodicity to ease feasibility search
    periodicity_scale = 0.3;
    ceq = [ceq;
           periodicity_scale * (x_start(2:10) - x_plus(2:10))] ;
end
