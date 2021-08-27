function [N1, N2] = basisNum(n, d, opt)
% >> This function calculates numbers of basis functions
% for the approximate optimal cost function \hat{V} and
% the approximate optimal control \hat{u}.
%
% The number of monomials composed of n variables from
% degree 1 to degree d is:
% \sum_{i=1}^{d} (i+n-1)! / (i!*(n-1)!).
%
% >> [N1, N2] = basisNum(n, d, opt)
% ======================== Input =========================
% n:   number of variables
% d:   approximation degree
%      The highest degree of monomials for \hat{u} is d.
%      The highest degree of monomials for \hat{V} is d+1.
% opt: type of basis functions
% ========================================================
% ======================== Output ========================
% N1:  number of basis functions for \hat{V}
% N2:  number of basis functions for \hat{u}
% ========================================================

switch opt
    case 'mono'
        N2 = 0;
        nFact = prod(1:n-1);
        for i = 1:d
            N2 = prod(1:i+n-1) / (prod(1:i)*nFact) + N2;
        end
        N1 = prod(1:d+n) / (prod(1:(d+1))*nFact) + N2;
    otherwise
        fprintf('Please choose the type of basis ');
        fprintf('functions.\n');
        fprintf('The type can be\n');
        fprintf('''mono''.\n\n');
end
end