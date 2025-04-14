function [Xout, Xinfo] = generate_feasible_points_LHS(controller_flag, nPool, nSelect, P)
% GENERATE_FEASIBLE_POINTS_LHS
%   Generates a pool of feasible design vectors (1x9, positive)
%   using Latin Hypercube Sampling (LHS).
%   Then selects a subset that are far apart in 9D space.
%
%   Inputs:
%     nPool   = number of feasible points desired
%     nSelect = number of final points chosen from the feasible pool
%     P       = open-loop plant used in cost_fun_basic
%
%   Output:
%     Xout    = nSelect x 9 array of chosen feasible starting points
%
%   Requirements:
%     - cost_fun_basic.m and create_PID_Notch_Controller.m must be on MATLAB path
%     - Statistics and Machine Learning Toolbox (for lhsdesign)

    % Example variable bounds (must be >0). Adjust to your needs:
    % Define Initial Guess and Bounds for PID Parameters and Notch filter
    %     P    I    D    T      LPF           Notch
    %                                    w1   w2       Q1     Q2  
    %    (1)  (2)  (3)  (4)     (5)      (6)  (7)      (8)   (9)
    %lowerB = [1, 1, 1 ,1 ,1 , 1 ,1 ,1e-3 ,1e-3];
    %upperB = [1e7, 1e4, 1e4, 1e4, 1e4, 2e3, 2e3, 5e1, 5e1];
    lowerB = 1;
    upperB = 1e5;

    switch lower(controller_flag)
        case 'pid'
            N = 3;
        case 'pidt'
            N = 4;
        case 'pidt lpf'
            N = 5;
        case 'pidt notch'
            N = 7;
        otherwise
            error('Unknown flag ''%s''.', controller_flag);
    end

    % --------------------------------------------------------------------
    % 1) Generate candidates with Latin Hypercube Sampling
    % --------------------------------------------------------------------
    % Often, we generate more LHS points than we actually need to ensure
    % enough feasible points. For example, sample 2*nPool total:
    nCandidates = 10 * nPool;

    % LHS returns an nCandidates x 9 matrix with values in (0, 1)
    lhsMatrix = lhsdesign(nCandidates, N);

    % Scale each column to [lowerB, upperB]
    %Xcand = lowerB + (upperB - lowerB) .* lhsMatrix;
    Xcand = 10.^( log10(lowerB) + (log10(upperB)-log10(lowerB)) .* lhsMatrix );

    % --------------------------------------------------------------------
    % 2) Evaluate feasibility of each candidate
    % --------------------------------------------------------------------
    feasiblePoints = [];
    feasible_fBW = [];       % To record the bandwidth (f_BW)
    feasible_stability = []; % To record the stability flag
    feasible_margins = [];

    for i = 1:nCandidates
        xCandidate = Xcand(i, :);

        % Build the candidate controller
        xCell = num2cell(xCandidate);        % Convert the numeric row to a cell array
        C_obj = PIDNotchController(controller_flag, xCell{:});

        % Evaluate the cost function
        [f_BW, validity_flag, stability, margins] = cost_fun_basic(P, C_obj.C);

        % If feasible, store it
        if validity_flag == 1
            feasiblePoints = [feasiblePoints; xCandidate];
            feasible_fBW = [feasible_fBW; f_BW];
            feasible_stability = [feasible_stability; stability];
            feasible_margins = [feasible_margins; margins];
            %fprintf('\n Margins: \n GM = %.1f PM = %.1f MM = %.1f \n Stable: %0.f \n Bandwidth: %.1f', [margins stability, f_BW]);
        end

        % Early break if we've reached enough feasible
        if size(feasiblePoints,1) >= nPool
            break
        end
    end
    
    % Check if we obtained enough feasible points
    if (size(feasiblePoints,1) < nPool)
        warning('Only found %d feasible points (need %d). Returning all feasible.', ...
            size(feasiblePoints,1), nPool);
    end

    % --------------------------------------------------------------------
    % 3) Select nSelect points with the highest recorded bandwidth (f_BW)
    % --------------------------------------------------------------------
    if size(feasiblePoints, 1) <= nSelect
        Xout = feasiblePoints;
        Xinfo = [feasible_fBW, feasible_stability, feasible_margins];
        return;
    else
        % Sort feasible candidates by f_BW in descending order so higher bandwidth comes first
        [~, idxSort] = sort(feasible_fBW, 'descend');
        selectedIdx = idxSort(1:nSelect);
    
        % Xout retains just the design parameters
        Xout = feasiblePoints(selectedIdx, :);
        % Xinfo contains corresponding [f_BW, stability, margins]
        Xinfo = [feasible_fBW(selectedIdx), feasible_stability(selectedIdx), feasible_margins(selectedIdx, :)];
    end
