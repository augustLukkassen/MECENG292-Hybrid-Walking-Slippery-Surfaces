function [value, isterminal, direction] = cont_slip_event(t,s, z, params)
    % Transition from Slip to Stick
    % Occurs when relative velocity of contact point becomes zero.
    
    % Check velocity of stance foot.
    s10 = s(1:10);
    dx_s = dpSt_gen(s10) ; 
    
    if length(dx_s) > 1
        val = dx_s(1); % Tangential velocity
    else
        val = dx_s;
    end

    if isfield(params, 'v_stick_tol')
        vtol = params.v_stick_tol;
    else
        vtol = 1e-3;
    end

    value = abs(val) - vtol;
    isterminal = 1 ; 
    direction = -1 ; 
end
