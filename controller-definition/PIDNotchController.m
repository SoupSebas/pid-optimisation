classdef PIDNotchController < handle
    % PIDNotchController 
    %   A unified class for creating PID or PID+tamed, optionally with
    %   a low-pass filter or a notch filter. The final transfer function
    %   is stored in the public property C.
    %
    %   Usage examples:
    %   1) obj = PIDNotchController('pid', P, I, D)
    %   2) obj = PIDNotchController('pidt', P, I, D, D_tamed)
    %   3) obj = PIDNotchController('pidt lpf', P, I, D, D_tamed, LPF)
    %   4) obj = PIDNotchController('pidt notch', P, I, D, D_tamed, w_notch, Q1, Q2)
    %
    %   The resulting controller TF is placed in obj.C

    properties
        C % The resulting controller transfer function
    end
    
    methods
        function obj = PIDNotchController(flag, varargin)
            % Constructor for the PIDNotchController object.
            %
            % flag (char): 
            %     'pid'          -> P, I, D
            %     'pidt'         -> P, I, D, D_tamed
            %     'pidt lpf'     -> P, I, D, D_tamed, LPF
            %     'pidt notch'   -> P, I, D, D_tamed, w_notch, Q1, Q2
            %
            % varargin: numeric inputs according to the chosen flag.

            switch lower(flag)
                
                case 'pid'
                    % Expect: P, I, D
                    if numel(varargin) ~= 3
                        error('Flag "pid" requires inputs: P, I, D');
                    end
                    P = varargin{1}; 
                    I = varargin{2};
                    D = varargin{3};
                    
                    % By user definition: D_tamed = 9*D
                    D_tamed = 9 * D;
                    % Low-pass corner: set it large or effectively no filter 
                    % if you want a "PID only" style

                    obj.C = definePID(P, I, D, D_tamed);

                case 'pidt'
                    % Expect: P, I, D, D_tamed
                    if numel(varargin) ~= 4
                        error('Flag "pidt" requires inputs: P, I, D, D_tamed');
                    end
                    P        = varargin{1};
                    I        = varargin{2};
                    D        = varargin{3};
                    D_tamed  = varargin{4};

                    obj.C = definePID(P, I, D, D_tamed);

                case 'pidt lpf'
                    % Expect: P, I, D, D_tamed, LPF
                    if numel(varargin) ~= 5
                        error('Flag "pidt lpf" requires inputs: P, I, D, D_tamed, LPF');
                    end
                    P        = varargin{1};
                    I        = varargin{2};
                    D        = varargin{3};
                    D_tamed  = varargin{4};
                    LPF      = varargin{5};

                    % Make the tamed PID
                    pid_t = definePID(P, I, D, D_tamed);
                    lpf = defineLPF(LPF);
                    obj.C = pid_t * lpf;  % The "product" is just the tamed PID + built-in LPF

                case 'pidt notch'
                    % Expect: P, I, D, D_tamed, w_notch, Q1, Q2
                    if numel(varargin) ~= 7
                        error('Flag "pidt notch" requires inputs: P, I, D, D_tamed, w_notch, Q1, Q2');
                    end
                    P        = varargin{1};
                    I        = varargin{2};
                    D        = varargin{3};
                    D_tamed  = varargin{4};
                    w_notch  = varargin{5};
                    Q1       = varargin{6};
                    Q2       = varargin{7};

                    % You can set your own default or large LPF corner if needed:
                    
                    % Create the tamed PID:
                    pid_t = definePID(P, I, D, D_tamed);

                    % Make the notch filter:
                    % Frequencies for the notch are the same w_notch, 
                    % as user requested.
                    notch_filter = defineNotch(w_notch, w_notch, Q1, Q2);

                    % Final controller is the product:
                    obj.C = pid_t * notch_filter;
                case 'pidt lpf notch'
                    % Expect: P, I, D, D_tamed, w_notch, Q1, Q2
                    if numel(varargin) ~= 8
                        error('Flag "pidt lpf notch" requires inputs: P, I, D, D_tamed, w_notch, Q1, Q2 LPF');
                    end
                    P        = varargin{1};
                    I        = varargin{2};
                    D        = varargin{3};
                    D_tamed  = varargin{4};
                    w_notch  = varargin{5};
                    Q1       = varargin{6};
                    Q2       = varargin{7};
                    LPF      = varargin{8};

                    % You can set your own default or large LPF corner if needed:
                    
                    % Create the tamed PID:
                    pid_t = definePID(P, I, D, D_tamed);

                    % Make the notch filter:
                    % Frequencies for the notch are the same w_notch
                    notch_filter = defineNotch(w_notch, w_notch, Q1, Q2);
                    
                    % create lowpassfilter
                    lpf = defineLPF(LPF);

                    % Final controller is the product:
                    obj.C = pid_t * notch_filter * lpf;

                otherwise
                    error('Unknown flag ''%s''.', flag);
            end

        end % constructor
    end
end
