function [f_BW, flag, stability, margins] = cost_fun_basic(P,C)
    S = 1/(1+P*C);
    margins = allmargin(P*C);
    
    margins.DMFrequency = margins.DMFrequency./(2*pi);
    margins.GainMargin = 20*log10(margins.GainMargin);
    
    if ~isempty(margins.GainMargin) && ~isempty(margins.PhaseMargin) && ~isempty(margins.DMFrequency)
        GM = margins.GainMargin(1,end);
        PM = margins.PhaseMargin(1,1);
        freq = logspace(-2, 8, 1000);
        MM = 20*log10(max(abs(freqresp(S, freq))));
        f_BW = margins.DMFrequency(1,1);
        CL = feedback(P*C,1);
        stability = isstable(CL);
    else
        GM = 3;
        PM = 0;
        MM = 7;
        f_BW = 0;
        stability = false;
    end

    if (GM >= 6) && (PM >= 40) && (MM <= 6) && (stability == true)
        flag = 1;
    else
        flag = 0;
    end
    margins = [GM PM MM];
end