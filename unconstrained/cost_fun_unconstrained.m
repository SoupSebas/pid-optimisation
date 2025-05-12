function [f_BW, f_weighted, GM, PM, MM] = cost_fun_unconstrained(P,C)

    margins = allmargin(P*C);
    S = 1/(1+P*C);

    margins.DMFrequency = margins.DMFrequency./(2*pi);
    margins.GainMargin = 20*log10(margins.GainMargin);
    if ~isempty(margins.GainMargin) && ~isempty(margins.PhaseMargin) && ~isempty(margins.DMFrequency)
        GM = margins.GainMargin(1,end);
        PM = margins.PhaseMargin(1,1);
        freq = logspace(-2, 8, 1000); 
        MM = 20*log10(max(abs(freqresp(S, freq))));
        f_BW = margins.DMFrequency(1,1);
    else
        GM = 3;
        PM = 0;
        MM = 7;
        f_BW = 0;
    end
    
    
    penalty = barrier_function(GM,PM,MM,100);
    f_weighted = f_BW - penalty;
    fprintf(['Bandwidth: %.0f | Penalty: %.0f | ' ...
             'Margins [GM PM MM]:  [%.0f | %.0f | %.0f] \n '], ...
             [f_weighted, penalty, GM, PM , MM]);
end