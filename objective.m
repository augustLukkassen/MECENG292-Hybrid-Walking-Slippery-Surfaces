function cost = objective(z, nx, nu, N)
    idx_X_end = nx * N ;
    idx_U_end = idx_X_end + nu * N ;
   
    U_vec = z(idx_X_end + 1 : idx_U_end) ; 

    cost = U_vec'*U_vec ; 
end