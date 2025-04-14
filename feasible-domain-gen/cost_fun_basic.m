function [f_BW, flag, stability, margins] = cost_fun_basic(P,C)
    freq = logspace(-2, 8, 1000);     
    S = 1/(1+P*C);
    margins = allmargin(P*C);
    
    margins.DMFrequency = margins.DMFrequency./(2*pi);
    margins.GainMargin = 20*log10(margins.GainMargin);
    f_BW = margins.DMFrequency(1,1);

    GM = margins.GainMargin(1,end);
    PM = margins.PhaseMargin(1,1);
    MM = 20*log10(max(abs(freqresp(S, freq))));

    CL = feedback(P*C,1);
    stability = isstable(CL);

    if (GM >= 6) && (PM >= 40) && (MM <= 6) && (stability == true)
        flag = 1;
    else
        flag = 0;
    end
    margins = [GM PM MM];
end