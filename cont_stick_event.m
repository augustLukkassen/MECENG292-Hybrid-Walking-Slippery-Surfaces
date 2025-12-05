function [value, isterminal, direction] = cont_stick_event(t,s)
    dx_s = dpSt_gen(s) ; 
    
    value = dx_s ; 
    isterminal = 1 ; 
    direction = 0 ;  %The direction of slip doesnt matter?
end

