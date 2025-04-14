%% Termination control


function stop = terminality_criteria_eval(z, optimValues, state, P, controller_flag)
    x = exp(z);
    persistent fvals feas_flags
    threshold = 1;
    N = 20;
    stop = false;
    
    switch state
        case 'init'
            fvals = [];
            feas_flags = [];
        case 'iter'
            % Store function value
            fvals(end+1) = optimValues.fval;

            % Evaluate feasibility

            % Create the PID
            C_inputs = num2cell(x);
            C_termination = PIDNotchController(controller_flag, C_inputs{:});
            C = C_termination.C;
            [~, stability_flag, ~, ~] = cost_fun_basic(P,C);
            feas_flags(end+1) = stability_flag;

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


function cost = obj_fun(z, P, controller_flag)
    x = exp(z);
    C_inputs = num2cell(x);
    C_obj = PIDNotchController(controller_flag, C_inputs{:});
    C = C_obj.C;
    [~, f_BW_weighted, ~, ~, ~] = cost_fun_unconstrained(P,C);

    cost = -1 * f_BW_weighted;
end

%% Script logic for optimiser

controller_flag = 'pidt';

fprintf('Loading plant... \n')
load("PlantTF_hard.mat");
fprintf('Plant loaded \n');
% Define Initial Guess and Bounds for PID Parameters and Notch filter
%     P    I    D    T      LPF           Notch
%                                    w1   w2       Q1     Q2  
%    (1)  (2)  (3)  (4)     (5)      (6)  (7)      (8)   (9)

fprintf('Creating feasible starting points... \n');
x0 = generate_feasible_points_LHS(controller_flag, 100,1, P);
z0 = log(x0);
fprintf('Feasible points created \n \n');


fprintf('P: %2f I: %2f D: %2f T: %2f LPF: %2f w1: %2f w2: %2f Q1: %2f Q2: %2f', x0);

options = optimset('OutputFcn', @(x, optimValues, state)terminality_criteria_eval(x, optimValues, state, P, controller_flag));
[z_opt,fval,exitflag,output] = fminsearch(@(z) obj_fun(z, P, controller_flag), z0, options);

%% Display Results
x_opt = exp(z_opt);  % if x is the same length as z


x_opt_cell = num2cell(x_opt);
fprintf(['Optimized Parameters: P = %.1f, I = %.1f, D = %.1f, D_tamed = %.1f\n ...' ...
'LPF = %.1f Notch: w1 = %.1f, w2 = %.1f, q1 = %.3f, q2 = %.3f \n'], [x_opt_cell{:}]);
fprintf('Maximum Bandwidth: %.1f Hz\n', -1*fval);
fprintf('\n Exit flag: %.0f', exitflag);

C_optimised_obj = PIDNotchController(controller_flag, x_opt_cell{:});
C_optimised = C_optimised_obj.C;
cost_fun_unconstrained(P,C_optimised);

figure;
opts = bode_options_helper(10,1e5);
bode(P*C_optimised, opts);
grid on;
title('Optimal Open-Loop Frequency Response');
