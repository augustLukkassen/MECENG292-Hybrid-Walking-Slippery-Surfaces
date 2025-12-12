function [value, isterminal, direction] = cont_slip_event(t,s, z, params)
    % Transition from Slip to Stick
    % Occurs when relative velocity of contact point becomes zero.
    
    % Check velocity of stance foot.
    dx_s = dpSt_gen(s) ; 
    
    if length(dx_s) > 1
        val = dx_s(1); % Tangential velocity
    else
        val = dx_s;
    end
    
    value = val ; 
    isterminal = 1 ; 
    direction = 0 ; 
end
