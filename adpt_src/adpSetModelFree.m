function adpOpt = adpSetModelFree(f1, v1, f2, v2, f3, v3,...
    f4, v4, f5, v5)
% >> This function specifies settings of the function
% adpModelFree.
%
% 1. Basis functions
% 1) The approximate optimal control is represented by
%    \hat{u} = w * Phi_u,
%    where Phi_u is a column vector composed of basis
%    functions, and w is a row vector or a matrix
%    (depending on the control dimension), composed of
%    coefficients.
% 2) The approximate optimal cost function is represented
%    by \hat{V} = Phi_V * c,
%    where Phi_V is a row vector composed of basis
%    functions, and c is a column vector, composed of
%    coefficients.
% 3) Supported basis functions:
%    'mono': monomials
%
% 2. ADP iterations.
% 1) The user may specify data to be used in the iteration
%    process by choosing the stride in each trajectory.
%    For example, suppose two trajectories are generated as
%    [x11; x12; ...; x1a; x21; x22; ...; x2b],
%    where x1~ indicates a value in the first trajectory
%    and x2~ indicates a value in the second trajectory.
%    The user may only use
%    x11, x13, x15, ...,
%    and
%    x21, x24, x27, ...,
%    by setting stride to [2; 3].
% 2) Coefficients are calculated through several
%    iterations. The stop criterion can be:
%    0: |c-c_old|   <= epsilon
%    1: |cw-cw_old| <= epsilon
%    2: |c-c_old|   <= epsilon * |c_old|
%    3: |cw-cw_old| <= epsilon * |cw_old|
%    The user may specify the stop criterion (crit) and
%    epsilon (epsilon).
% 3) The user may choose the maximum number of iterations
%    (maxIter).
%
% >> adpOpt = adpSetSysFree(f1, v1, f2, v2, f3, v3,
%       f4, v4, f5, v5)
% ======================== Input =========================
% f1, ..., f5: name of some specific setting
% v1, ..., v5: value to be assigned to f1, ..., f5
%
% The number of input can be 0, 2, 4, 6, 8 or 10.
%
% Supported settings:
% basisOpt: type of basis functions
%           default: 'mono'
% stride:   For example, suppose two trajectories are
%           generated as
%           [x11; x12; ...; x1a; x21; ...; x2b],
%           where x1~ indicates a value in the first
%           trajectory and x2~ indicates a value in the
%           second trajectory. The user may only use
%           x11, x13, x15, ...,
%           and
%           x21, x24, x27, ...,
%           by setting stride to [2; 3].
%           default: [1; 1; ...] (all data will be used)
% crit:     stop criterion (0 / 1 / 2 / 3)
%           default: 1
% epsilon:  epsilon in the stop criterion
%           default: 0.001
% maxIter:  maximum iteration times
%           default: 100
% ========================================================
% ======================== Output ========================
% adpOpt:     settings of the function adpMainSysFree
%             (struct with fields)
% ========================================================


% For ADP iterations.
BASIS_FUNCTION = 'mono'; % type of basis functions
STRIDE          = [];
STOP_CRITERION = 1; % stop criterion
EPSILON        = 0.001;
ITERATION_MAX  = 100; % maximum iteration times

% All options for ADP are contained in adpOpt.
adpOpt.opt     = BASIS_FUNCTION;
adpOpt.stride  = STRIDE;
adpOpt.crit    = STOP_CRITERION;
adpOpt.epsilon = EPSILON;
adpOpt.maxIter = ITERATION_MAX;

% Valid options.
SETTINGS = {'basisOpt', 'stride', 'crit', 'epsilon', 'maxIter'};

switch nargin
    case 0
        fprintf('Default settings would be used.\n');
    case 2
        adpOpt = adpAssign(adpOpt,SETTINGS,f1,v1);
    case 4
        adpOpt = adpAssign(adpOpt,SETTINGS,f1,v1);
        adpOpt = adpAssign(adpOpt,SETTINGS,f2,v2);
    case 6
        adpOpt = adpAssign(adpOpt,SETTINGS,f1,v1);
        adpOpt = adpAssign(adpOpt,SETTINGS,f2,v2);
        adpOpt = adpAssign(adpOpt,SETTINGS,f3,v3);
    case 8
        adpOpt = adpAssign(adpOpt,SETTINGS,f1,v1);
        adpOpt = adpAssign(adpOpt,SETTINGS,f2,v2);
        adpOpt = adpAssign(adpOpt,SETTINGS,f3,v3);
        adpOpt = adpAssign(adpOpt,SETTINGS,f4,v4);
    case 10
        adpOpt = adpAssign(adpOpt,SETTINGS,f1,v1);
        adpOpt = adpAssign(adpOpt,SETTINGS,f2,v2);
        adpOpt = adpAssign(adpOpt,SETTINGS,f3,v3);
        adpOpt = adpAssign(adpOpt,SETTINGS,f4,v4);
        adpOpt = adpAssign(adpOpt,SETTINGS,f5,v5);
    otherwise
        fprintf('Invalid arguments for the function ');
        fprintf('adpSet.\n');
        fprintf('Default settings would be used.');
end

end


function adpOpt = adpAssign(adpOpt, SETTINGS, f, v)
% >> This function assigns some setting a new value.
%
% >> adpOpt = adpAssign(adpOpt, SETTINGS, f, v)
% ======================== Input =========================
% adpOpt:   current settings for ADP
%           (struct with fields)
% SETTINGS: a set containing all valid options for ADP
% f:        field (option) to be assigned
% v:        value that would be assigned to the field f
% ========================================================
% ======================== Output ========================
% adpOpt:   settings for ADP after the assignment
%           (struct with fields)
% ========================================================

if any(strcmp(SETTINGS,f))
    adpOpt.(f) = v;
else
    showMsg(f);
end
end


function showMsg(f)
% >> This function displays some information about the
% assignment of settings of ADP.
%
% ======================== Input =========================
% f: specific field of the struct adpOpt
% ========================================================

fprintf('Error: Unrecognized property name ''');
fprintf('%s', f);
fprintf('''.\n');
end