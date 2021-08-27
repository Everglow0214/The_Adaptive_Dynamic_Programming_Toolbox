function A = vec2symmat(v, N)
% >> This function converts a vector to a symmetric matrix.
%
% If the symmetric matrix restored is given by
% [a1, a2, a3;
%  a2, a4, a5;
%  a3, a5, a6],
% then the vector is given by
% [a1; a2; a4; a3; a5; a6],
% which saves upper triangular elements of the matrix.
%
% The idea is from
% https://www.mathworks.com/matlabcentral/answers/
% 94562-how-can-i-specify-a-symmetric-matrix-so-that-
% matlab-does-not-need-to-create-and-store-the-redundant.
%
% >> A = vec2symmat(v, N)
% ======================== Input =========================
% v: a vector that saves upper triangular elements of some
%    symmetric matrix
%    v is a column vector.
% N: The size of the symmetric matrix is N * N.
% ========================================================
% ======================== Output ========================
% A: the symmetric matrix restored from v
% ========================================================

% Restore upper triangular elements.
A = zeros(N, N);
idx = [0, cumsum(1:N)];
for i = 1:N
    A(1:i, i) = v(idx(i)+1: idx(i+1));
end

% Restore the whole sysmetric matrix.
A = A + A' - eye(N) .* A;
end