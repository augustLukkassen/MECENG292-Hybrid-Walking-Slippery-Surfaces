function [x_plus, next_mode] = impact(x_minus, params)
    lambda = Fimpact_gen(x_minus) ;
    lambda_x = lambda(1) ; 
    lambda_y = lambda(2) ; 
    mu = params.mu ; 

    if abs(lambda_x) < mu*lambda_y
        % Stick Impact
        dq_plus = dqPlus_gen(x_minus) ;
        next_mode = "stick"; 
    else
        % Slip Impact
        dq_plus = dqPlus_s_gen(x_minus) ; 
        next_mode = "slip";
    end 
    
    q_minus = x_minus(1:5);
    R = eye(5) ; 
    R(3:4,3:4) = [0 1; 1 0] ;
    
    x_plus =  [R*q_minus; R*dq_plus] ;
end

