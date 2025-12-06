function [x_plus, t] = impact(x_minus,t, params)
    lambda = Fimpact(x_minus) ;
    lambda_x = lambda(1) ; 
    lambda_y = lambda(2) ; 
    mu = params.mu ; 

    if lambda_x < mu*lambda_y
        x_plus = dqPlus(x_minus) ;
    else
        x_plus = dqPlus_s(x_minus) ; 
    end 

end