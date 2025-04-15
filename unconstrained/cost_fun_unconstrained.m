function [f_BW, f_weighted, GM, PM, MM] = cost_fun_unconstrained(P,C)

    margins = allmargin(P*C);
    S = 1/(1+P*C);

    margins.DMFrequency = margins.DMFrequency./(2*pi);
    margins.GainMargin = 20*log10(margins.GainMargin);

    GM = margins.GainMargin(1,end);
    PM = margins.PhaseMargin(1,1);
    freq = logspace(-2, 8, 1000); 
    MM = 20*log10(max(abs(freqresp(S, freq))));
    
    f_BW = margins.DMFrequency(1,1);
    penalty = barrier_function(GM,PM,MM,100);
    f_weighted = f_BW - penalty;
    %fprintf('\n Bandwidth = %.0f \n Penalty = %.0f', [f_weighted, penalty]);
end