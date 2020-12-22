function adpOpt = adpSetModelBased(f1,v1,f2,v2,f3,v3,f4,v4,...
                           f5,v5,f6,v6,f7,v7,f8,v8,f9,v9,...
                           f10,v10,f11,v11,f12,v12,...
                           f13,v13,f14,v14,f15,v15)
% >> This function specifies settings of the function
% adpModelBased.
%
% 1. Initial states
% 1) Two default initial states are generated randomly, with
%    the value of each initial state in an open interval
%    (xInitMin, xInitMax) or (-xInitMax, -xInitMin).
% 2) The user may choose the number (xInitNum) and the range
%    (xInitMin and xInitMax) of initial states.
% 3) The user may also directly specify initial state(s)
%    (xInit) by a row vector or a matrix with each row
%    indicating one initial condition.
% 4) Trajectories will be calculated starting from these
%    initial states for a time interval [0, tSpan], where
%    tSpan is settable.
% 5) Trajectories are calculated by the Runge-Kutta method.
%    Settings of the Runge-Kutta method can be modified by
%    calling the function odeSet.
%
% 2. Initial control
% 1) The default initial control is calculated from the
%    linear quadratic method after linearization around the
%    origin.
% 2) The user may also directly specify the initial control
%    (u0Symb) in a symbolic form.
%
% 3. Exploration signals
% 1) Default exploration noise signals are sums of four
%    sinusoidal signals with different frequencies.
% 2) The user may specify the number (numFreq) and the
%    amplitude (explAmpl) of sinusoidal signals in one
%    exploration signal.
% 3) The number of exploration signals automatically matches
%    with the control dimension and the number of initial
%    states.
% 4) All of these default exploration signals are different.
% 5) The user may also directly specify exploration signals
%    (explSymb).
%
% 4. Basis functions
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
% 5. ADP iterations.
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
% >> adpOpt = adpSetSysBased(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,
%                            f6,v6,f7,v7,f8,v8,f9,v9,f10,v10,
%                            f11,v11,f12,v12,f13,v13,f14,v14,
%                            f15,v15)
% ========================== Input ===========================
% f1, f2, ..., f15: name of some specific setting
% v1, v2, ..., v15: value to be assigned to f1, f2, ..., f15
%
% The number of input can be 0, 2, 4, 6, ..., 28 or 30.
%
% Supported settings:
% xInit:    initial state(s)
%           Each initial state is represented by a row vector.
%           If 2 or more initial states are used, please stack
%           these row vectors vertically into a matrix.
%           default: [] (temporarily)
% xInitNum: number of initial states
%           default: 2
% xInitMin: minimum of each element in a default initial state
%           default: 0.3
% xInitMax: maximum of each element in a default initial state
%           default: 0.9
% tSpan:    time interval(s) for exploration
%           If only 1 initial state is used, please represent
%           tSpan by a row vector. If 2 or more initial states
%           are used, please stack these row vectors
%           vertically into a matrix.
%           default: [0, 8;
%                      ...;
%                     0, 8] (size: xInitNum*2)
% odeOpt:   options for the ODE solver ode45
%           default: odeset('RelTol', 1e-6, 'AbsTol', 1e-6)
% u0Symb:   initial control
%           The value for u0Symb should be symbolic.
%           default: [] (temporarily)
% explSymb: exploration noise signal
%           The value for explSymb should be symbolic.
%           default: [] (temporarily)
% explAmpl: amplitude of sinusoidal signals in exploration
%           noise signals
%           default: 0.8
% numFreq:  number of sinusoidal signals in each exploration
%           noise signal
%           default: 4
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
% adpOpt:     settings of the function 'adpMainSysBased'
%             (a struct with fields)
% ============================================================