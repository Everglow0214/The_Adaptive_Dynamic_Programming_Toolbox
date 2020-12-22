function adpOpt = adpSetModelFree(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5)
% >> This function specifies settings of the function
% adpModelFree.
%
% 1. Basis functions
% 1) The approximate optimal control is represented by
%    \hat{u} = w * Phi_u,
%    where Phi_u is a column vector composed of basis
%    functions, and w is a row vector or a matrix (depending
%    on the control dimension), composed of coefficients.
% 2) The approximate optimal cost function is represented by
%    \hat{V} = Phi_V * c,
%    where Phi_V is a row vector composed of basis functions,
%    and c is a column vector, composed of coefficients.
% 3) Supported basis functions:
%    'mono': monomials
%
% 2. ADP iterations.
% 1) The user may specify data to be used in the iteration
%    process by choosing the stride in each trajectory.
%    For example, suppose two trajectories are generated as
%    [x11; x12; ...; x1a; x21; x22; ...; x2b],
%    where x1~ indicates a value in the first trajectory and
%    x2~ indicates a value in the second trajectory. The user
%    may only use x11, x13, x15, ..., and x21, x24, x27, ...,
%    by setting stride to [2; 3].
% 2) Coefficients are calculated through several iterations.
%    The stop criterion can be:
%    0: |c-c_old|   <= epsilon
%    1: |cw-cw_old| <= epsilon
%    2: |c-c_old|   <= epsilon * |c_old|
%    3: |cw-cw_old| <= epsilon * |cw_old|
%    The user may specify the stop criterion (crit) and
%    epsilon (epsilon).
% 3) The user may choose the maximum number of iterations
%    (maxIter).
%
% >> adpOpt = adpSetSysFree(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5)
% ========================== Input ===========================
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
%           trajectory and x2~ indicates a value in the second
%           trajectory. The user may only use
%           x11, x13, x15, ..., and x21, x24, x27, ...,
%           by setting stride to [2; 3].
%           default: [1; 1; ...] (all data will be used)
% crit:     stop criterion (0 / 1 / 2 / 3)
%           default: 1
% epsilon:  epsilon in the stop criterion
%           default: 0.001
% maxIter:  maximum iteration times
%           default: 100
% ============================================================
% ========================== Output ==========================
% adpOpt:     settings of the function 'adpMainSysFree'
%             (struct with fields)
% ============================================================