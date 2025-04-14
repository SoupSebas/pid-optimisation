%% Termination control


function stop = terminality_criteria_eval(x, optimValues, state, P)
    persistent fvals feas_flags
    threshold = 1e-2;
    N = 50;
    stop = false;
    
    switch state
        case 'init'
            fvals = [];
            feas_flags = [];
        case 'iter'
            % Store function value
            fvals(end+1) = optimValues.fval;

            % Evaluate feasibility
            C_termination = create_PID_Notch_Controller(x);
            [~, flag] = cost_fun_basic(P, C_termination);  % assumes C is controller x-dependent
            feas_flags(end+1) = flag;

            % Once we have N steps:
            if length(fvals) >= N
                delta = abs(diff(fvals(end-N+1:end)));
                avg_improvement = mean(delta);

                % Check feasibility of last N
                recent_flags = feas_flags(end-N+1:end);
                all_feasible = all(recent_flags == 1);

                % Terminate if both conditions met
                if avg_improvement < threshold && all_feasible
                    stop = true;
                end
            end
        case 'done'
            clear fvals feas_flags
    end
end

%% Objective function


function cost = obj_fun(x, P)

    C_obj = PIDNotchController('pidt', C_inputs{:});
C = C_obj.C;
    [~, f_BW_weighted, ~, ~, ~] = cost_fun_unconstrained(P,C);

    cost = -1 * f_BW_weighted;
end

%% Script logic for optimiser


load("PlantTF.mat");
P = G;
clear G;

% Define Initial Guess and Bounds for PID Parameters and Notch filter
%     P    I    D    T      LPF           Notch
%                                    w1   w2       Q1     Q2  
%    (1)  (2)  (3)  (4)     (5)      (6)  (7)      (8)   (9)
flag = 'pidt';
x0 = generate_feasible_points_LHS(flag, 1000,1, P);



fprintf('P: %2f I: %2f D: %2f T: %2f LPF: %2f w1: %2f w2: %2f Q1: %2f Q2: %2f', x0);

options = optimset('OutputFcn', @(x, optimValues, state)terminality_criteria_eval(x, optimValues, state, P));
[x_opt,fval,exitflag,output] = fminsearch(@(x) obj_fun(x, P), x0, options);

%% Display Results



fprintf(['Optimized Parameters: P = %.1f, I = %.1f, D = %.1f, D_tamed = %.1f\n ...' ...
'LPF = %.1f Notch: w1 = %.1f, w2 = %.1f, q1 = %.3f, q2 = %.3f\n'], ...
x_opt(1), x_opt(2), x_opt(3), x_opt(4), x_opt(5), x_opt(6), x_opt(7), x_opt(8), x_opt(9));
fprintf('Maximum Bandwidth: %.3f Hz\n', -1*fval);

C_optimised = create_PID_Notch_Controller(x_opt);
cost_fun_unconstrained(P,C_optimised);

figure;
opts = bode_options_helper(10,1e5);
bode(P*C_optimised, opts);
grid on;
title('Optimal Open-Loop Frequency Response');
