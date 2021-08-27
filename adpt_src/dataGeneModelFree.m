function [data, N] = dataGeneModelFree(t, x, u0, e, m,...
    N2, q, R, opt)
% >> This function evaluates integrals that would be used
% in the iteration process by the trapezoidal method in
% the model-free working mode.
%
% >> [data, N] = dataGeneModelFree(t, x, u0, e, m, N2, q,
%       R, opt)
% ======================== Input =========================
% t:    time recorded
%       t is a column vector.
%       For every consecutive time interval, the starting
%       time should be 0 or should be shifted to 0.
%       For example, t may be like
%       [t11; t12; ...; t1a; t21; t22; ...t2b],
%       where t1x and t2x are non-consecutive time
%       intervals and t11 = t21 = 0.
% x:    state values recorded
%       x is a column vector or a matrix, with each row
%       indicating a state value at the corresponding
%       time that is recorded in t.
% u0:   control inputs recorded
%       u0 is a column vector or a matrix, with each row
%       indicating a control value at the corresponding
%       time that is recorded in t.
% e:    exploration signals recorded
%       e is a column vector or a matrix, with each row
%       indicating an exploration signal value at the
%       corresponding time that is recorded in t.
% m:    control dimension
% N2:   number of basis functions for the approximate
%       optimal control \hat{u}
% q:    q(x) in the performance index
%       (function handle)
% R:    matrix R in the performance index
% opt:  type of basis functions
%       This parameter can be 'mono'.
% ========================================================
% ======================== Output ========================
% data: integrals evaluated by the trapezoidal method that
%       would be used in the iteration process
% N:    number of integrals for every consecutive time
%       interval
%       N is a scalar or a column vector.
% ========================================================


num = size(x,1);
idx = find(t==0);
idxx = [idx(2:end); num+1];
N = idxx - idx;

%%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%
% temp = zeros(num, N2*(N2+2*m)+2);
%%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%

%%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%
temp = zeros(num, N2*(N2/2+0.5+2*m)+2);
%%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%

u = u0 + e;

switch opt
    case 'mono'
        for i = 1:num
            Phi_u = mono_u(x(i,:),N2);
            
            % Generate necessary data to be used in the
            % trapezoidal numerical integration.
            
            %%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%
%             x_cost = q(x(i,:)'); % scalar
%             u0_cost = u0(i,:)*R*u0(i,:)'; % scalar
%             
%             alpha = kron(R*e(i,:)', Phi_u); % size: (m*N2) * 1
%             beta = kron(R*u(i,:)', Phi_u); % size: (m*N2) * 1
%             gamma = kron(Phi_u, Phi_u); % size: N2^2 * 1
            %%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%
            x_cost = q(x(i,:)'); % scalar
            u0_cost = u0(i,:) * R * u0(i,:)'; % scalar
            
            alpha = vec(R * e(i,:)' * Phi_u'); % size: (m*N2) * 1
            beta = vec(R * u(i,:)' * Phi_u'); % size: (m*N2) * 1
            
            gamma = zeros(N2*(N2+1)/2, 1);
            idx_start = [0, cumsum(1:N2)];
            for j = 1:N2
                gamma(idx_start(j)+1: idx_start(j+1)) =...
                    Phi_u(1:j) * Phi_u(j);
            end
            %%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%
            
            temp(i,:) = [x_cost, u0_cost, alpha', beta', gamma'];
        end
    otherwise
        fprintf('Please choose the type of basis functions.\n');
        fprintf('The type can be\n');
        fprintf('''mono''.\n\n');
end

% Trapezoidal numerical integration.
for i = 1:numel(N)
    idx1 = sum(N(1:i-1))+1;
    idx2 = sum(N(1:i));
    temp(idx1:idx2,:) = cumtrapz(t(idx1:idx2),temp(idx1:idx2,:),1);
end

data = [x, temp];
end