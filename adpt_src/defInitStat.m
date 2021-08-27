function xInit = defInitStat(n, xInitNum, xInitMin, xInitMax)
% >> This function generates initial states to be used in
% ode45.
%
% Every initial state is represented by a row vector. If 2
% or more initial states are generated, then these row
% vectors are stacked vertically into a matrix.
%
% For example, the output may be like
% x11 x12 x13 ... x1n -> 1st initial state
% x21 x22 x23 ... x2n -> 2nd initial state
% x31 x32 x33 ... x3n -> 3rd initial state
%
% >> xInit = defInitStat(n, xInitNum, xInitMin, xInitMax)
% ======================== Input =========================
% n:        state dimension
% xInitNum: number of initial states
% xInitMin: minimum of each component in an initial state
% xInitMax: maximum of each component in an initial state
% ========================================================
% ======================== Output ========================
% xInit:    initial states
%           xInit is a row vector or a matrix.
% ========================================================

xInit = (xInitMax-xInitMin) * rand(xInitNum,n) + xInitMin;

% Generate a matrix that is in the form
%  1 -1  1 -1  1 -1 ...
% -1  1 -1  1 -1  1 ...
% ...
initCoe = sign(mod(1:n,2)-0.5) .* sign(mod(1:xInitNum,2)-0.5)';

xInit = xInit .* initCoe;

end