function Phi_V = mono_V(x, N1)
% The approximate optimal cost function is represented by
% \hat{V} = Phi_V * c,
% where Phi_V is a row vector composed of monomial bases,
% and c is a column vector, composed of coefficients.
%
% >> This function generates \hat{V}, a row vector
% composed of monomials for the approximate optimal cost
% function.
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
% >> Phi_V = mono_V(x, N1)
% ======================== Input =========================
% x:     a (row or column) vector of variables
% N1:    number of monomial bases
% ========================================================
% ======================== Output ========================
% Phi_V: a row vector composed of monomial bases
% Phi_V = [x1, x2, ..., xn, x1*x1, x1*x2, ...].
% size(Phi_V) = 1 * N1.
% ========================================================

% size(Phi_V) = 1 * N1
Phi_V = mono_u(x,N1)';
end