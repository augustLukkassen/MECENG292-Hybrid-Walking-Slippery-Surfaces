function [value, isterminal, direction] = cont_stick_event(t,s, z, params)
    % Transition from Stick to Slip
    % Occurs when friction constraint is violated: |F_x| > mu * F_z
    
    mode = "stick";
    u = io_linearization(t, s, params, mode);
    
    lambda = Fst_gen(s, u) ; 
    lambda_x = lambda(1) ; 
    lambda_z = lambda(2) ; 
    mu = params.mu ; 
    
    % Event triggers when value = 0.
    % We want to trigger when |F_x| - mu*F_z goes from negative (valid) to positive (invalid).
    
    value = abs(lambda_x) - mu*lambda_z; 
    isterminal = 1 ; 
    direction = 1 ; 
end
