function [u0sym, K] = defInitCtrl(fsym, gsym, xsym,...
    usym, qsym, n, m, R)
% >> This function generates a default initial admissible
% control for a system that is in the form
% \dot{x} = f(x) + g(x)*u.
%
% First, the system is linearized around the origin. Then,
% the control calculated by LQR is used as the default
% control.
%
% >> [u0, K] = initCtrl(fsym, gsym, xsym, usym, qsym, n,
%       m, R)
% ======================== Input =========================
% fsym:  f(x) in the system dynamics
%        (symbolic representation)
% gsym:  g(x) in the system dynamics
%        (symbolic representation)
% xsym:  system state
%        (symbolic representation)
% usym:  control input
%        (symbolic representation)
% qsym:  q(x) in the performance index
%        (symbolic representation)
% n:     state dimension
% m:     control dimension
% R:     matrix R in the performance index
% ========================================================
% ======================== Output ========================
% u0sym: default initial control
%        (symbolic representation)
% K:     feedback gain by LQR
% ========================================================

% Linearization.
Asym = jacobian(fsym+gsym*usym, xsym);
Bsym = jacobian(fsym+gsym*usym, usym);

A = subs(Asym, [xsym;usym], zeros(n+m,1));
A = double(A);
B = subs(Bsym, [xsym;usym], zeros(n+m,1));
B = double(B);

% Check if the linearized system can be stabilized.
% The pair (A,B) is stabilizable if and only if
% rank([kI-A B]) = n for all Re(k)>=0.
eigVal = eig(A);
for i = 1:numel(eigVal)
    if eigVal(i) >= 0
        if rank([eigVal(i)*eye(n)-A, B]) < n
            fprintf('The linearized system cannot be ');
            fprintf('stabilized.\n');
            fprintf('Please choose an initial admissible ');
            fprintf('control manually.\n')
            break
        end
    end
end

% Half of the Hessian matrix of q(x) at the origin is
% chosen as the default Q matrix for LQR.
Qsym = hessian(qsym, xsym);
Q = subs(Qsym, xsym, zeros(n,1));
Q = double(Q)/2;

K = lqr(A,B,Q,R);
u0sym = -K*xsym;
end