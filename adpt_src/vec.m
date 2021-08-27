function y = vec(x)
% >> This function vectorizes a matrix.
%
% If the original matrix is x = [x1 x2 ... xn] whose size
% is m*n, the generated vector is
% y = [x1; x2; ...; xn], whose size is mn*1.
%
% >> y = vec(x)
% ======================== Input =========================
% x: original matrix
% ========================================================
% ======================== Output ========================
% y: vector after vectorization
% ========================================================

[a,b] = size(x);
y = reshape(x, a*b, 1);
end