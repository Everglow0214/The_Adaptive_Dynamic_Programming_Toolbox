function dX = adpAugSys(t, X, sys, u0, e, q, n, N2, R, opt)
% The performance index is
% \int_{0}^{\infty} q(x) + u'*R*u dt.
%
% >> This function is to be used in ode45 to collect data.
%
% >> dX = adpAugSys(t, X, sys, u0, e, q, n, N2, R, opt)
% ======================== Input =========================
% t:   time
% X:   augmented system states
% sys: system dynamics
%      (function handle)
% u0:  initial admissible control
%      (function handle)
% e:   exploration signal
%      (function handle)
% q:   penalty term in the performance index
%      (function handle)
% n:   state dimension
% N2:  number of basis functions for the approximate
%      optimal control \hat{u}
% R:   matrix R in the performance index
% opt: type of basis functions
%      This parameter can be 'mono'.
% ========================================================
% ======================== Output ========================
% dX:  derivatives of augmented system states
% ========================================================

x = X(1:n);
u = u0(x) + e(t);
dx = sys(x,u);

% \hat{u} = w * Phi_u
% size(w):     m * N2
% size(Phi_u): N2 * 1
    
% ============================================
% In case other basis functions will be added.
% ============================================

switch opt
    case 'mono'
        Phi_u = mono_u(x,N2);
    otherwise
            fprintf('Please choose the type of basis');
            fprintf(' functions.\n');
            fprintf('The type can be\n');
            fprintf('''mono''.\n\n');
end

%%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%
% x_cost = q(x);
% u0_cost = u0(x)'*R*u0(x);
% alpga = kron(R*e(t), Phi_u); % size: (m*N2) * 1
% beta = kron(R*u, Phi_u); % size: (m*N2) * 1
% gamma = kron(Phi_u, Phi_u); % size: N2^2 * 1
%%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%

%%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%
x_cost = q(x);
u0_cost = u0(x)' * R * u0(x);
alpha = vec(R * e(t) * Phi_u'); % size: (m*N2) * 1
beta = vec(R * u * Phi_u'); % size: (m*N2) * 1

gamma = zeros(N2*(N2+1)/2, 1);
idx = [0, cumsum(1:N2)];
for i = 1:N2
    gamma(idx(i)+1: idx(i+1)) = Phi_u(1:i) * Phi_u(i);
end
%%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%

dX = [dx; x_cost; u0_cost; alpha; beta; gamma];
end