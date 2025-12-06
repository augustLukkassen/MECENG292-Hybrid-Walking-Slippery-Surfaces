function J = objective(z, N, nx, nu, na, nb)

    % unpack
    x  = reshape(z(1:N*nx), nx, N);                           % states
    u  = reshape(z(N*nx+1 : N*nx+N*nu), nu, N);               % controls
    alpha = z(N*nx+N*nu+1 : N*nx+N*nu+na);                    % alpha
    beta  = z(N*nx+N*nu+na+1 : N*nx+N*nu+na+nb);              % beta
    T     = z(end);                                           % step time

    h = T/(N-1);

    % simple Simpson-like cost on u'u
    J = 0 ;
    for k = 1:N-1
        uk   = u(:,k);
        uk1  = u(:,k+1);
        umid = 0.5*(uk + uk1);   

        J = J + (h/6)*(uk.'*uk + 4*umid.'*umid + uk1.'*uk1);
    end
end
