function [w, c, adpOpt] = adpModelFree(t, x, n, u0, m, e,...
    d, q, R, adpOpt)
% >> This is the main function in the model-free working mode
% of the ADPT.
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
% >> [w, c, adpOpt] = adpModelFree(t, x, n, u0, m, e, d,
%       q, R)
%    [w, c, adpOpt] = adpModelFree(t, x, n, u0, m, e, d,
%       q, R, adpOpt)
% ======================== Input =========================
% t:      time recorded
%         t is a column vactor.
%         For every consecutive time interval, the starting
%         time should be 0 or should be shifted to 0.
%         For example, t may be like
%         [t11; t12; t13; ...; t1a; t21; t22; ...; t2b; ...],
%         where t1x and t2x are non-consecutive time
%         intervals and t11 = t21 = 0.
% x:      state values recorded
%         x is a column vector or a matrix, with each row
%         indicating a state value at the corresponding
%         time that is recorded in t.
% n:      state dimension
%         n is same as the number of columns of x.
% u0:     control inputs recorded
%         u0 is a column vector or a matrix, with each
%         row indicating a control value at the
%         corresponding time that is recorded in t.
% m:      control dimension
%         m is same as the number of columns of u0.
% e:      exploration signals recorded
%         e is a column vector or a matrix, with each
%         row indicating an exploration signal value at
%         the corresponding time that is recorded in t.
%         The number of columns of e is m.
% d:      approximation degree
%         The highest degree of monomials for \hat{u} is
%         d.
%         The highest degree of monomials for \hat{V} is
%         d+1.
% q:      q(x) in the performance index
%         (function handle)
%         For example, q(x) can be defined as
%         q = @(x) 2*x(1)^2 + 3*x(2)^2
% R:      R matrix in the performance index
% adpOpt: settings of the function adpModelFree
% ========================================================
% ======================== Output ========================
% w:      w in \hat{u} = w * Phi_u
% c:      c in \hat{V} = Phi_V * c
% adpOpt: settings of the function adpModelFree
%         In case that default options are used, the user
%         may output this term to check details.
% ========================================================

switch nargin
    case 9
        adpOpt = adpSetModelFree();
        fprintf('Working...\n\n');
    case 10
        fprintf('Working...\n\n');
    otherwise
        fprintf('Please check inputs of the function adpMain.\n');
end

% Define some basic parameters.
stride  = adpOpt.stride;
iterMax = adpOpt.maxIter;
crit    = adpOpt.crit;
epsilon = adpOpt.epsilon;
opt     = adpOpt.opt;

% Number of basis functions to approximate V and u.
[N1, N2] = basisNum(n, d, opt);

% Generate approximate integrals.
[data, N] = dataGeneModelFree(t, x, u0, e, m, N2, q, R, opt);
if isempty(stride)
    stride = ones(numel(N),1);
    adpOpt.stride = stride;
end

if numel(stride) == 1
    stride = stride*ones(numel(N),1);
    adpOpt.stride = stride;
end

[Phi_V, Costx, Costu0, alpha, beta, gamma] = dataProc(data,...
    N, stride, n, N1, m, N2, opt);

% ADP iterations.
[w, c] = adpIter(iterMax, N1, N2, R, Phi_V, Costx, Costu0,...
    alpha, beta, gamma, epsilon, crit);

% Create 2 .m files which are uAdp.m and VAdp.m.
create_uAdp(n, m, N2, w);
create_VAdp(n, N1, c);

% Symbolic form of the approximate control.
% To be compatible with earlier MATLAB versions, variables
% x_u_extend and x_v_extend are created.
x = sym('x', [n,1]);
assume(x, 'real');

x_u_extend = ones(N2,1) * x';
Du = basisMono(n,N2);
uAdpSym = w * prod(x_u_extend .^ Du, 2);
uAdpSymDec = vpa(uAdpSym, 4);
% Symbolic form of the approximate value function.
DV = basisMono(n,N1);
x_v_extend = ones(N1,1) * x';
VAdpSym = c' * prod(x_v_extend .^ DV, 2);
VAdpSymDec = vpa(VAdpSym, 4);

fprintf('The approximate optimal control u(x) ');
fprintf('(shown with 4 significant digits):\n\n');
disp(uAdpSymDec);
fprintf('The approximate optimal cost function V(x) ');
fprintf('(shown with 4 significant digits):\n\n');
disp(VAdpSymDec);

fprintf('The files uAdp.m and VAdp.m have been generated.\n\n');
fprintf('The approximate optimal control can be applied by calling\n');
fprintf('>> u = uAdp(x)\n');
fprintf('where x is the state vector.\n\n');
fprintf('Similarly, the approximate optimal cost function can be ');
fprintf('applied by calling\n');
fprintf('>> V = VAdp(x)\n\n');

% Create a .txt file.
create_txt(m, uAdpSym, uAdpSymDec, VAdpSym, VAdpSymDec);

% c = c';

end


function create_uAdp(n, m, N2, w)
% >> This function creates a .m file and save the approximated
% optimal control as a function.

fid = fopen('uAdp.m', 'wt');
fprintf(fid, '%s', 'function u = uAdp(x)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'N2 = ');
fprintf(fid, '%d', N2);
fprintf(fid, '%s', ';');
fprintf(fid, '\n');
fprintf(fid, '%s', 'w = [');
fprintf(fid, repmat('%f ',1,m*N2), w);
fprintf(fid, '%s', '];');
fprintf(fid, '\n');
fprintf(fid, '%s', 'w = reshape(w,[],N2);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'u = w * mono(x,N2);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n\n\n');

fprintf(fid, '%s', 'function Phi_u = mono(x,N2)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'n = ');
fprintf(fid, '%d', n);
fprintf(fid, '%s', ';');
fprintf(fid, '\n');
fprintf(fid, '%s', 'x = reshape(x,1,[]);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'x = ones(N2,1) * x;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'monoMatrix = x .^ basisMono(n,N2);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'Phi_u = prod(monoMatrix,2);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n\n\n');

fprintf(fid, '%s', 'function D = basisMono(n,num)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'D = zeros(num,n);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'r = zeros(1,n);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'i = 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'while i <= num');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r = monoNext(n,r);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    D(i,:) = r;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    i = i + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n\n\n');

fprintf(fid, '%s', 'function r = monoNext(n,r)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'j = n;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'for i = n-1:-1:1');
fprintf(fid, '\n');
fprintf(fid, '%s', '    if r(i) > 0');
fprintf(fid, '\n');
fprintf(fid, '%s', '        j = i;');
fprintf(fid, '\n');
fprintf(fid, '%s', '        break');
fprintf(fid, '\n');
fprintf(fid, '%s', '    end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'if j == n');
fprintf(fid, '\n');
fprintf(fid, '%s', '    temp = r(n);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(n) = 0;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(1) = temp + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'elseif j > 1');
fprintf(fid, '\n');
fprintf(fid, '%s', '    temp = r(n);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(j) = r(j) - 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(n) = 0;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(j+1) = temp + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'else');
fprintf(fid, '\n');
fprintf(fid, '%s', '    temp = r(n);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(n) = 0;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(1) = r(1) - 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(2) = temp + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fclose(fid);

%{
% This way is very slow =n=
% =========================
fid = fopen('uAdp.m', 'wt');
fprintf(fid, '%s', 'function u = uAdp(state)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'n = ');
fprintf(fid, '%d', n);
fprintf(fid, '%s', ';');
fprintf(fid, '\n');
fprintf(fid, '%s', 'syms x [n,1] real');
fprintf(fid, '\n');
fprintf(fid, '%s', 'uSym = [');
for i = 1:m-1
    fprintf(fid, '%s', uAdpSym(i,1));
    fprintf(fid, '%s', ';');
    fprintf(fid, '\n');
end
fprintf(fid, '%s', uAdpSym(m,1));
fprintf(fid, '%s', '];');
fprintf(fid, '\n');
fprintf(fid, '%s', 'ux = matlabFunction(uSym, ''Vars'', {x});');
fprintf(fid, '\n');
fprintf(fid, '%s', 'u = ux(state);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fclose(fid);
% =========================
%}
end

function create_VAdp(n, N1, c)
% >> This function creates a .m file and save the approximated
% optimal cost function as a function.
fid = fopen('VAdp.m', 'wt');
fprintf(fid, '%s', 'function V = VAdp(x)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'N1 = ');
fprintf(fid, '%d', N1);
fprintf(fid, '%s', ';');
fprintf(fid, '\n');
fprintf(fid, '%s', 'c = [');
fprintf(fid, repmat('%f ',1,N1), c);
fprintf(fid, '%s', '];');
fprintf(fid, '\n');
fprintf(fid, '%s', 'V = c * mono(x,N1);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n\n\n');

fprintf(fid, '%s', 'function Phi_V = mono(x,N1)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'n = ');
fprintf(fid, '%d', n);
fprintf(fid, '%s', ';');
fprintf(fid, '\n');
fprintf(fid, '%s', 'x = reshape(x,1,[]);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'x = ones(N1,1) * x;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'monoMatrix = x .^ basisMono(n,N1);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'Phi_V = prod(monoMatrix,2);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n\n\n');

fprintf(fid, '%s', 'function D = basisMono(n,num)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'D = zeros(num,n);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'r = zeros(1,n);');
fprintf(fid, '\n');
fprintf(fid, '%s', 'i = 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'while i <= num');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r = monoNext(n,r);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    D(i,:) = r;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    i = i + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n\n\n');

fprintf(fid, '%s', 'function r = monoNext(n,r)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'j = n;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'for i = n-1:-1:1');
fprintf(fid, '\n');
fprintf(fid, '%s', '    if r(i) > 0');
fprintf(fid, '\n');
fprintf(fid, '%s', '        j = i;');
fprintf(fid, '\n');
fprintf(fid, '%s', '        break');
fprintf(fid, '\n');
fprintf(fid, '%s', '    end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'if j == n');
fprintf(fid, '\n');
fprintf(fid, '%s', '    temp = r(n);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(n) = 0;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(1) = temp + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'elseif j > 1');
fprintf(fid, '\n');
fprintf(fid, '%s', '    temp = r(n);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(j) = r(j) - 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(n) = 0;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(j+1) = temp + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'else');
fprintf(fid, '\n');
fprintf(fid, '%s', '    temp = r(n);');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(n) = 0;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(1) = r(1) - 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', '    r(2) = temp + 1;');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fprintf(fid, '\n');
fprintf(fid, '%s', 'end');
fclose(fid);
end

function create_txt(m, uAdpSym, uAdpSymDec, VAdpSym, VAdpSymDec)
% >> This function creates a .txt file and saves the approximate
% optimal control and the approximate optiam cost function in
% symbolic forms.

fid = fopen('readme.txt', 'wt');
fprintf(fid, '%s', 'The symbolic appoximate optimal control in');
fprintf(fid, '%s', ' fraction form:');
fprintf(fid, '\n');
for i = 1:m
    fprintf(fid, '%s', char(uAdpSym(i)));
    fprintf(fid, '\n');
end
fprintf(fid, '\n');
fprintf(fid, '%s', 'The symbolic appoximate optimal control in');
fprintf(fid, '%s', ' decimal form:');
fprintf(fid, '\n');
for i = 1:m
    fprintf(fid, '%s', char(uAdpSymDec(i)));
    fprintf(fid, '\n');
end

fprintf(fid, '\n');
fprintf(fid, '%s', 'The symbolic approximate optimal cost');
fprintf(fid, '%s', ' function in fraction form:');
fprintf(fid, '\n');
fprintf(fid, '%s', char(VAdpSym));
fprintf(fid, '\n\n');
fprintf(fid, '%s', 'The symbolic approximate optimal cost');
fprintf(fid, '%s', ' function in decimal form:');
fprintf(fid, '\n');
fprintf(fid, '%s', char(VAdpSymDec));
fprintf(fid, '\n\n');

fprintf(fid, '%s', 'The files uAdp.p and VAdp.p have been');
fprintf(fid, '%s', ' generated.');
fprintf(fid, '\n\n');
fprintf(fid, '%s', 'The approximate optimal control can be');
fprintf(fid, '%s', ' applied by calling');
fprintf(fid, '\n');
fprintf(fid, '%s', '>> u = uAdp(x)');
fprintf(fid, '\n');
fprintf(fid, '%s', 'where x is the state vector.');
fprintf(fid, '\n\n');
fprintf(fid, '%s', 'Similarly, the approximate optimal cost');
fprintf(fid, '%s', ' function can be applied by calling');
fprintf(fid, '\n');
fprintf(fid, '%s', '>> V = VAdp(x)');
fclose(fid);
end