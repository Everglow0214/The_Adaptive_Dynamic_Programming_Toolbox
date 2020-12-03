function [w,c,adpOpt] = adpModelFree(t,x,n,u0,m,e,d,q,R,adpOpt)
% >> This is the main function in the model-free working mode
% of the ADPT.
%
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.
%
% The approximate optimal control is represented by
% \hat{u} = w * Phi_u,
% where Phi_u is a column vector composed of basis
% functions, and w is a row vector or a matrix (depending
% on the control dimension), composed of coefficients.
%
% The approximate optimal cost function is represented by
% \hat{V} = Phi_V * c,
% where Phi_V is a row vector composed of basis functions,
% and c is a column vector, composed of coefficients.
%
% >> [w,c,adpOpt] = adpMainSysFree(t,x,n,u0,m,e,d,q,R)
%    [w,c,adpOpt] = adpMainSysFree(t,x,n,u0,m,e,d,q,R,adpOpt)
% ========================== Input ===========================
% t:      time recorded
%         t is a column vactor.
%         For every consecutive time interval, the starting
%         time should be 0 or should be shifted to 0.
%         For example, t may be like
%         [t11; t12; t13; ...; t1a; t21; t22; ...; t2b; ...],
%         where t1x and t2x are non-consecutive time intervals
%         and t11 = t21 = 0.
% x:      state values recorded
%         x is a column vector or a matrix, with each row
%         indicating a state value at the corresponding time
%         that is recorded in t.
% n:      state dimension
%         n is same as the number of columns of x.
% u0:     control inputs recorded
%         u0 is a column vector or a matrix, with each row
%         indicating a control value at the corresponding time
%         that is recorded in t.
% m:      control dimension
%         m is same as the number of columns of u0.
% e:      exploration signals recorded
%         e is a column vector or a matrix, with each row
%         indicating an exploration signal value at the
%         corresponding some time that is recorded in t.
%         The number of columns of e is m.
% d:      approximation degree
%         The highest degree of monomials for \hat{u} is
%         d.
%         The highest degree of monomials for \hat{V} is
%         d+1.
% q:      q(x) in the performance index
%         (function handle)
%         For example, q(x) can be defined as
%         q = @(x) 2*x(1)^2 + 3*x(2)^2
% R:      R matrix in the performance index
% adpOpt: settings of the function adpModelFree
% ============================================================
% ========================== Output ==========================
% w:      w in \hat{u} = w * Phi_u
% c:      c in \hat{V} = Phi_V * c
% adpOpt: settings of the function adpModelFree
%         In case that default options are used, the user
%         may output this term to check details.
% ============================================================