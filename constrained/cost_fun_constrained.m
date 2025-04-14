function [f_BW,flag, GM, PM, MM] = cost_fun_constrained(P,C)

    margins = allmargin(P*C);
    S = 1/(1+P*C);

    
    % Display results

    margins.DMFrequency = margins.DMFrequency./(2*pi);
    margins.GainMargin = 20*log10(margins.GainMargin);
    if isempty(margins.GainMargin)
        GM = -Inf;
    else
        GM = margins.GainMargin(1,end);
    end

    if isempty(margins.PhaseMargin)
        PM = -Inf;
    else
        PM = margins.PhaseMargin(1,1);
    end

    if isempty(margins.DMFrequency)
        f_BW = -Inf;
    else
        f_BW = margins.DMFrequency(1,1);
    end

    freq = logspace(-2, 8, 1000); 
    MM = 20*log10(max(abs(freqresp(S, freq))));
    
    
    if (GM >= 6) && (PM >= 40) && (MM <= 6)
        flag = 1;
    else
        flag = 0;
    end
end