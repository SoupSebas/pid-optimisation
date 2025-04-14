function penalty = barrier_function(GM,PM, MM,r_factor)
    %penalty =  0;
    %penalty = penalty - (1/r_factor)*log(-GM + 6) ...
    %                  - (1/r_factor)*log( PM + 140) ...
    %                  - (1/r_factor)*log( MM - 6);


    if (GM <= 6) || (PM <= 40) || (MM >= 6)
        penalty = Inf;
    else
        GM_penalty = (1/r_factor)*(1/(GM-6)^2);
        PM_penalty = (1/r_factor)*(1/(PM-40)^2);
        MM_penalty = (1/r_factor)*(1/(6-MM)^2);
        penalty = GM_penalty + PM_penalty +  MM_penalty;
    end
end