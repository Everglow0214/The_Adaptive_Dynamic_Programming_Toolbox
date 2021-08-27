function dX = adpAppSys(~, X, sys, q, n, N2, w, R, opt)
% The performance index is
% \int_{0}^{\infty} q(x) + u'*R*u dt.
%
% The cumulative cost is
% \int_{0}^{T} q(x) + u'*R*u dt.
%
% The approximate optimal control is represented by
% \hat{u} = w * Phi_u.
%
% >> This function is to be used in ode45 to test the
% approximate optimal control.
%
% >> dX = adpAppSys(~, X, sys, q, n, N2, w, R, opt)
% ======================== Input =========================
% ~:   time
% X:   augmented system states, including system states
%      and the cumulative cost
% sys: system dynamics
%      (function handle)
% q:   penalty term in the performance index
%      (function handle)
% n:   state dimension
% N2:  number of basis functions for the approximate
%      optimal control \hat{u}
% w:   converged coefficients for the approximate optimal
%      control \hat{u}
% R:   matrix R in the performance index
% opt: type of basis functions
%      This parameter can be 'mono'.
% ========================================================
% ======================== Output ========================
% dX:  derivatives of augmented system states,
%      including dx and (q(x) + u'*R*u)
% ========================================================

x = X(1:n);

% \hat{u} = w * Phi_u
% size(w):     m * N2
% size(Phi_u): N2 * 1

switch opt
    case 'mono'
        Phi_u = mono_u(x,N2);
    otherwise
            fprintf('Please choose the type of basis');
            fprintf(' functions.\n');
            fprintf('The type can be\n');
            fprintf('''mono''.\n\n');
end

u = w * Phi_u;

dx = sys(x,u);

dcost = q(x) + u'*R*u;
dX = [dx; dcost];

end