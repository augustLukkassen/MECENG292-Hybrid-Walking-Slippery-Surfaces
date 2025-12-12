function [value, isterminal, direction] = slip_events(t,s, z, params)
    % Impact Event 
    [v_imp, ist_imp, dir_imp] = impact_event(t,s) ; 

    % Continous Slip Event
    [v_cont, ist_cont, dir_cont] = cont_slip_event(t,s, z, params) ; 

    value = [v_imp; v_cont] ; 
    isterminal = [ist_imp; ist_cont] ; 
    direction = [dir_imp; dir_cont] ; 
end
