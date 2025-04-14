function stop = optim_plot(x, optimValues, state, P)
    % Declare global logs for accepted iterates
    global accepted_log accepted_nlog;  
    persistent lastLoggedIteration

    % Initialize persistent variable on first call
    if isempty(lastLoggedIteration)
        lastLoggedIteration = -1;
    end

    % Only log when state is 'iter' and the iteration count has increased
    if strcmp(state, 'iter') && optimValues.iteration > lastLoggedIteration
        lastLoggedIteration = optimValues.iteration;
        
        % Compute objective function value (f_BW) and stability metrics 
        % NOTE: Ensure P is available in the workspace (e.g., declared global)
        C = create_PID_Notch_Controller(x);
        [f_BW, ~, GM, PM, MM] = cost_fun_constrained(P, C);
        
        % Log the accepted iterate: store design variables and corresponding f_BW
        accepted_log = [accepted_log; x(:)', f_BW];  % row: [x1, x2, ..., x9, f_BW]
        
        % Log the stability metrics: GM, PM, MM
        accepted_nlog = [accepted_nlog; GM, PM, MM];
    end

    % --- Plotting (using the accepted logs) ---
    figure(100); clf;
    
    % Create three subplots: PID gains, bandwidth, stability metrics
    subplot(3,1,1);
    if ~isempty(accepted_log)
        % For illustration, plot the first parameter (e.g., P gain) normalized
        plot(accepted_log(:,1), 'r-o','LineWidth',1.5); hold on;
        plot(accepted_log(:,2), 'm-o','LineWidth',1.5); 
        plot(accepted_log(:,3), 'g-o','LineWidth',1.5); 
        plot(accepted_log(:,5), 'c-o','LineWidth',1.5); 
        plot(accepted_log(:,6), 'b-o','LineWidth',1.5); 
        plot(accepted_log(:,7), 'b-o','LineWidth',1.5); 
        title('Accepted PID Gain (P) Over Iterations'); xlabel('Iteration'); ylabel('P');
        grid on;
    end
    
    subplot(3,1,2);
    if ~isempty(accepted_log)
        % Plot bandwidth (stored as the last column of accepted_log)
        plot(accepted_log(:,end), 'b-o','LineWidth',1.5);
        title('Accepted Bandwidth Over Iterations'); xlabel('Iteration'); ylabel('Bandwidth (Hz)');
        grid on;
    end
    
    subplot(3,1,3);
    if ~isempty(accepted_nlog)
        plot(accepted_nlog(:,1), 'm-o','LineWidth',1.5); hold on;
        plot(accepted_nlog(:,2), 'g-o','LineWidth',1.5);
        plot(accepted_nlog(:,3), 'k-o','LineWidth',1.5);
        legend('GM (dB)', 'PM (deg)', 'MM (dB)');
        title('Accepted Stability Metrics'); xlabel('Iteration'); ylabel('Value');
        grid on;
        hold off;
    end
    drawnow;

    stop = false;  % Continue optimization
end

%% Objective function


function cost = obj_fun(y, P, s)
    x = 10.^y;
    C = create_PID_Notch_Controller(x);
    [f_BW, ~, ~, ~, ~] = cost_fun_constrained(P,C);
    cost = -1 * f_BW;
end


%% Nonlinear constraints
function [c, ceq] = nonlcon(y, P, s)
    x = 10.^y;
    C = create_PID_Notch_Controller(x);
    [~, ~, GM, PM, MM] = cost_fun_constrained(P,C);

    % Stability & Performance Constraints
    c(1) = 6 - GM;  % Ensure Gain Margin ≥ 6 dB
    c(2) = 40 - PM; % Ensure Phase Margin ≥ 40°
    c(3) = MM - 6;  % Ensure Peak Sensitivity ≤ 6 dB

    % No equality constraints
    ceq = [];
end


%% Abstraction layer and housekeeping
clear accepted_log accepted_nlog lastLoggedIteration;
clf;

load("PlantTF.mat");
P = G;
clear G;

%% Script logic for optimiser
% Define Initial Guess and Bounds for PID Parameters and Notch filter
%     P    I    D    T      LPF           Notch
%                                    w1   w2       Q1     Q2  
%    (1)  (2)  (3)  (4)     (5)      (6)  (7)      (8)   (9)
x0 = [200,100, 200, 1e4,    1e4,    800,  800,    0.1,   50];  

lb = [1,    1,   1,   1,     10,     400,  400,  1e-1, 1e-1];  
ub = [1e3, 1e4, 1e4, 1e4,    1e5,    1200, 1200,   100,  100];  

% Scaling of variables for better results
s = 1;  % for instance, scale each variable by its upper bound

% Transform initial guess
y0 = log10(x0); 

% Redefine bounds in y-space: they become  lb_y = lb ./ s, ub_y = ub ./ s
lb_y = log10(lb);
ub_y = log10(ub);

% Optimization Options
options = optimoptions('fmincon', 'Algorithm', 'sqp', ...
    'Display', 'iter', ...
    'OutputFcn', @(x,optimValues,state) optim_plot(x,optimValues,state,P));

% Run `fmincon()` Optimization
[y_opt, fval,exitflag,output] = fmincon(@(x) obj_fun(x, P, s), x0, [], [], [], [], lb_y, ub_y, @(x) nonlcon(x, P, s), options);
x_opt = 10.^y_opt;



%% Display Results
fprintf(['Optimized Parameters: P = %.3f, I = %.3f, D = %.3f, D_tamed = %.3f\n ...' ...
'LPF = %.3f Notch: w1 = %.3f, w2 = %.3f, q1 = %.5f, q2 = %.5f\n'], ...
x_opt(1), x_opt(2), x_opt(3), x_opt(4), x_opt(5), x_opt(6), x_opt(7), x_opt(8), x_opt(9));
fprintf('Maximum Bandwidth: %.3f Hz\n', -fval);

%%Plot Final Frequency Response
%PIDF = definePIDF(x_opt(1), x_opt(2), x_opt(3), x_opt(4), x_opt(5));
%notch = defineNotch(x_opt(6), x_opt(7), x_opt(8), x_opt(9));

C_optimised = create_PID_Notch_Controller(x_opt);

figure;
opts = bode_options_helper(10,1e5);
bode(P*C_optimised, opts);
grid on;
title('Optimal Open-Loop Frequency Response');
