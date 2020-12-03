function [w,c,adpOpt] = adpModelBased(fsym,gsym,xsym,n,usym,...
                                      m,qsym,R,tsym,d,adpOpt)
% >> This is the main function in the model-based working
% mode of the ADPT.
%
% The system model is given by
% \dot{x} = f(x) + g(x)*u.
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
% >> [w,c,adpOpt] = adpModelBased(fsym,gsym,xsym,n,usym,m,
%                                 qsym,R,tsym,d)
%    [w,c,adpOpt] = adpModelBased(fsym,gsym,xsym,n,usym,m,
%                                 qsym,R,tsym,d,adpOpt)
% ======================== Input =========================
% fsym:   f(x) in the system dynamics
%         (symbolic representation)
% gsym:   g(x) in the system dynamics
%         (symbolic representation or just a matrix)
% xsym:   system state
%         (symbolic representation)
% n:      state dimension
% usym:   control input
%         (symbolic representation)
% m:      control dimension
% qsym:   q(x) in the performance index
%         (symbolic representation)
% R:      R matrix in the performance index
% tsym:   time
%         (symbolic representation)
% d:      approximation degree
%         The highest degree of monomials for \hat{u} is
%         d.
%         The highest degree of monomials for \hat{V} is
%         d+1.
% adpOpt: settings of the function adpModelBased
% ========================================================
% ======================== Output ========================
% w:      w in \hat{u} = w * Phi_u
% c:      c in \hat{V} = Phi_V * c
% adpOpt: settings of the function adpModelBased
%         In case that default options are used, the user
%         may output this term to check details.
% ========================================================