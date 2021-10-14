function [w, c] = adpIter(iterMax, N1, N2, R, Phi_V,...
    Costx, Costu0, alpha, beta, gamma, epsilon, crit)
% The approximate optimal control is represented by
% \hat{u} = w * Phi_u.
%
% The approximate optimal cost function is represented by
% \hat{V} = Phi_V * c.
%
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.
%
% >> This function executes the iteration process in ADP.
%
% >> [w, c] = adpIter(iterMax, m, N1, N2, R, Phi_V, Costx,
%       Costu0, alpha, beta, gamma, epsilon, crit)
% ======================== Input =========================
% iterMax: maximum number of iterations
% N1:      number of basis functions for the approximate
%          optimal cost function \hat{V}
% N2:      number of basis functions for the approximate
%          optimal control \hat{u}
% R:       matrix R in the performance index
% Phi_V ~ gamma:
%          outputs of the function dataProc
% epsilon: threshold
% crit:    stop criterion
%          crit can be 0, 1, 2, or 3.
%          0: |c-c_old|   <= epsilon
%             (used in RADP book)
%          1: |cw-cw_old| <= epsilon
%          2: |c-c_old|   <= epsilon*|c_old|
%          3: |cw-cw_old| <= epsilon*|cw_old|
% ========================================================
% ======================== Output ========================
% w:       w in \hat{u} = w * Phi_u
% c:       c in \hat{V} = Phi_V * c
% ========================================================


%%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%
% for i = 1: iterMax
%     if i == 1
%         A = [Phi_V, 2*alpha];
%         b = -(Costx + Costu0);
%     else
%         A = [Phi_V, 2*beta-2*gamma*kron((R*w_old)',eye(N2))];
%         b = -(Costx + gamma*vec(w_old'*R*w_old));
%     end
%%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%

%%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%
num = size(Costx, 1);
%%%%% With for loop later %%%%%
% gamma_full = zeros(num*N2, N2);
% for j = 1:num
%     gamma_full((j-1)*N2+1:j*N2, :) = vec2symmat(gamma(j,:)', N2);
% end
%%%%% With for loop later %%%%%

%%%%% Without for loop later %%%%%
gamma_full = zeros(N2, num*N2);
for j = 1:num
    gamma_full(:, (j-1)*N2+1:j*N2) = vec2symmat(gamma(j,:)', N2);
end
%%%%% Without for loop later %%%%%

clear gamma

for i = 1: iterMax
    if i == 1
        A = [Phi_V, 2*alpha];
        b = -(Costx + Costu0);
    else
%%%%% With for loop %%%%%
%         A_right = zeros(num, m*N2);
%         b_right = zeros(num, 1);
%         for j = 1:num
%             A_right(j,:) = vec(R * w_old *...
%                 gamma_full((j-1)*N2+1:j*N2, :))';
%             b_right(j) = sum((w_old' * R * w_old).*...
%                 gamma_full((j-1)*N2+1:j*N2, :), [1,2]);
%         end
%%%%% With for loop %%%%%

%%%%% Without for loop %%%%%
        A_right = reshape(R * w_old * gamma_full, [], num)';
        b_right = squeeze(sum(sum((w_old' * R * w_old) .*...
            reshape(gamma_full, N2, N2, num), 1), 2));
        
%         b_right = reshape(sum((w_old' * R * w_old).*...
%             reshape(gamma_full, N2, N2, num), [1,2]), num, 1);
%%%%% Without for loop %%%%%

        A = [Phi_V, 2*(beta-A_right)];
        b = -(Costx + b_right);
    end
%%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%

    % Solve the least square problem.
    cw = lsqlin(A,b);
    
    c = cw(1:N1);
    w = cw(N1+1:end);
    
    if i > 1
        % The criterion used in RADP book.
        % |c-c-old| <= epsilon
        if crit == 0
            if norm(c-c_old) <= epsilon % converged
                w = reshape(w, [], N2);
                showMsg(1, i);
                break
            else
                if i == iterMax % not converged
                    showMsg(0, i);
                end
            end
            
        % |cw-cw_old| <= epsilon
        elseif crit == 1
            if norm(cw-cw_old) <= epsilon % converged
                w = reshape(w, [], N2);
                showMsg(1, i);
                break
            else
                if i == iterMax % not converged
                    showMsg(0, i);
                end
            end
            
        % |c-c_old| <= epsilon*|c_old|
        elseif crit == 2
            if norm(c-c_old) <= epsilon*norm(c_old) % converged
                w = reshape(w, [], N2);
                showMsg(1,i);
                break
            else
                if i == iterMax % not converged
                    showMsg(0,i);
                end
            end
            
        % |cw-cw_old| <= epsilon*|cw_old|
        elseif crit == 3
            if norm(cw-cw_old) <= epsilon*norm(cw_old) % converged
                w = reshape(w, [], N2);
                showMsg(1,i);
            else
                if i == iterMax % not converged
                    showMsg(0,i);
                end
            end
        else
            fprintf('The stop criterion is invalid.\n');
        end
    end
    
    c_old = c;
    %%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%
    % w_old = reshape(w, N2, []);
    % w_old = w_old';
    %%%%%%%%%%%%%%% With Kronecker product %%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%
    w_old = reshape(w, [], N2);
    %%%%%%%%%%%%%%% No Kronecker product %%%%%%%%%%%%%%%
    cw_old = cw;
end
end

function showMsg(done,i)
% >> This function displays some information about the
% iteration process.
%
% ======================== Input =========================
% done: converged: 1
%       not converged: 0
% i:    number of iterations
% ========================================================

if done
    fprintf('Solved!\n');
    fprintf('Number of iterations: %d.\n\n', i);
else
    fprintf('Unsolved!\n');
    fprintf('Number of iterations: %d.\n', i);
    fprintf('Please try to:\n');
    fprintf('1. choose other initial states;\n');
    fprintf('2. choose other exploration signals;\n')
    fprintf('3. or choose a larger maximum number of ');
    fprintf('iterations (not recommended).\n');
end
end
