function C = defineLPF(LPF)
    s = tf('s');
    C = 1/((s/LPF) + 1);
end

    
    