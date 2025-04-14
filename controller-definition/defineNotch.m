function C = defineNotch(w_1, w_2, Q_1, Q_2)
    s = tf('s');
    C = ((s/w_1)^2 + (s/(w_1 * Q_1)) + 1) /...
        ((s/w_2)^2 + (s/(w_2 * Q_2)) + 1);
end