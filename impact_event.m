function [value, isterminal, direction] = impact_event(t,s)
    persistent gen_path_added
    if isempty(gen_path_added)
        root = fileparts(mfilename('fullpath'));
        gen_dir = fullfile(root, 'gen');
        if exist(gen_dir, 'dir')
            addpath(gen_dir);
        end
        gen_path_added = true;
    end

    s10 = s(1:10);
    pSw = pSw_gen(s10);
    pSt = pSt_gen(s10);
    dpSw = dpSw_gen(s10);

    tol = 1e-4;
    x_min = 0.75;
    if (pSw(1) - pSt(1)) <= x_min || dpSw(2) >= 0
        value = 1;
    else
        value = pSw(2) - tol;
    end
    isterminal = 1;
    direction = -1;
end

