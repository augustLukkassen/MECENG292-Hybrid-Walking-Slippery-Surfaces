function cost = objective(z, params) 
    try
        [t, x, ~] = simulate_single_step(z, params) ;

        if isempty(t) 
           cost = 1e9 ;
           return;
        end

        integral_cost = x(end, 11) ;
        distance_traveled = x(end, 1) ; %- z(1) ; 

        if abs(distance_traveled) < 1e-6 
            cost = 1e9 ; 
        else
            cost = integral_cost / distance_traveled ; 
        end
        
    catch
        cost = 1e9 ; 
    end
end