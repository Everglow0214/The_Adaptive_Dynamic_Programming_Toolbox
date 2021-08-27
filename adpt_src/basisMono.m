function D = basisMono(n, num)
% Monomials are selected as basis functions.
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
% >> This function generates a matrix with each element
% indicating the degree of some variable.
%
% For example, the output may be like
% 1 0 0 -> x1
% 0 1 0 -> x2
% 0 0 1 -> x3
% 2 0 0 -> x1*x1
% 1 1 0 -> x1*x2
% 1 0 1 -> x1*x3
% 0 2 0 -> x2*x2
% 0 1 1 -> x2*x3
% 0 0 2 -> x3*x3
%
% >> D = basisMono(n, num)
% ======================== Input =========================
% n:   number of variables
% num: number of monomials whose degree >= 1
% ========================================================
% ======================== Output ========================
% D:   matrix that indicates the degree of each
%      variable
% ========================================================


D = zeros(num,n);
r = zeros(1,n);

i = 1;
while i <= num
    r = monoNext(n,r);
    D(i,:) = r;
    i = i + 1;
end
end

function r = monoNext(n, r)
% >> This function generates the next row given the
% current row in the degree matrix.
%
% For example, if the current row is
% 2 0 0 -> x1*x1,
% the next row is
% 1 1 0 -> x1*x2.
%
% The idea is from
% https://people.sc.fsu.edu/~jburkardt/m_src/monomial/
% monomial.html.
%
% >> r = monoNext(n, r)
% ======================== Input =========================
% n: number of variables
% r: current row
% ========================================================
% ======================== Output ========================
% r: next row
% ========================================================

j = n;
for i = n-1:-1:1
    if r(i) > 0
        j = i;
        break
    end
end

% 3 situations
if j == n
    temp = r(n);
    r(n) = 0;
    r(1) = temp + 1;
elseif j > 1
    temp = r(n);
    r(j) = r(j) - 1;
    r(n) = 0;
    r(j+1) = temp + 1;
else % j == 1
    temp = r(n);
    r(n) = 0;
    r(1) = r(1) - 1;
    r(2) = temp + 1;
end
end