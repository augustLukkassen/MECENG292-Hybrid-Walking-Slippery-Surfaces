function [value, isterminal, direction] = impact_event(t,s)
    q1 = s(3) ; 
    q3 = s(5) ; 
    th1 = q1 + q3 - pi ; 

    value = th1 - pi/8 ; 
    isterminal = 1 ; 
    direction = 1 ; 
end

