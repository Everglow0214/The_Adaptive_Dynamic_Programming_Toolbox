function Phi_u = mono_u(x, N2)
% The approximate optimal control is represented by
% \hat{u} = w * Phi_u,
% where Phi_u is a column vector composed of monomial
% bases, and w is a row vector or a matrix (depending on
% the control dimension), composed of coefficients.
%
% >> This function generates \hat{u}, a column vector
% composed of monomials for the approximate optimal
% control.
%
% These monomials are ordered in the following way.
% x1, x2, x3, ..., xn,
% x1*x1, x1*x2, x1*x3, ..., x1*xn,
% x2*x2, ..., x2*xn, x3*x3, ..., xn*xn,
% x1*x1*x1, x1*x1*x2, x1*x1*x3, ..., x1*x1*xn,
% x1*x2*x2, x1*x2*x3, ..., x1*x2*xn, ..., x1*xn*xn,
% x2*x2*x2, x2*x2*x3, ..., x2*x2*xn, ..., xn*xn*xn,
% ....
%
% The number of monomials composed of n variables from
% degree 1 to degree d is:
% \sum_{i=1}^{d} (i+n-1)! / (i!*(n-1)!).
%
% >> Phi_u = mono_u(x, N2)
% ======================== Input =========================
% x:     a (row or column) vector of variables
% N2:    number of monomial bases
% ========================================================
% ======================== Output ========================
% Phi_u: a column vector composed of monomial bases
% Phi_u = [x1; x2; ...; xn; x1*x1; x1*x2; ...].
% size(Phi_u) = N2 * 1.
% ========================================================

% Reshape x as a row vector.
x = reshape(x,1,[]);
n = numel(x); % state dimension

% For element-wise power.
x = ones(N2,1) * x;

% size(monoMatrix) = N2 * n
monoMatrix = x .^ basisMono(n,N2);

% size(Phi_u) = N2 * 1
Phi_u = prod(monoMatrix,2);
end