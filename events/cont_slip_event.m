function [value, isterminal, direction] = cont_slip_event(t,s,params)
    u = io_linearization(s) ; 
    lambda = Fst_gen(s, u) ; 
    lambda_x = lambda(1) ; 
    lambda_z = lambda(2) ; 
    mu = params.mu ; 
    % Double check the x vs z 

    
    value = abs(lambda_x) - mu*lambda_z; 
    isterminal = 1 ; 
    direction = 1 ; 
end

