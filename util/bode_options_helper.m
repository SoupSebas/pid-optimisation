function opts = bode_options_helper(x0,x1)
opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.MagUnits = 'dB';
opts.Grid = 'on';
opts.XLim = [x0 x1];
end