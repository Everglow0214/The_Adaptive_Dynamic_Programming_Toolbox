function [w, c, adpOpt] = adpModelBased(fsym, gsym,...
    xsym, n, usym, m, qsym, R, tsym, d, adpOpt)
% >> This is the main function in the model-based working
% mode of the ADPT.
%
% The system model is given by
% \dot{x} = f(x) + g(x)*u.
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
% >> [w, c, adpOpt] = adpModelBased(fsym, gsym, xsym, n,
%       usym, m, qsym, R, tsym, d)
%    [w, c, adpOpt] = adpModelBased(fsym, gsym, xsym, n,
%       usym, m, qsym, R, tsym, d, adpOpt)
% ======================== Input =========================
% fsym:   f(x) in the system dynamics
%         (symbolic representation)
% gsym:   g(x) in the system dynamics
%         (symbolic representation or just a matrix)
% xsym:   system state
%         (symbolic representation)
% n:      state dimension
% usym:   control input
%         (symbolic representation)
% m:      control dimension
% qsym:   q(x) in the performance index
%         (symbolic representation)
% R:      R matrix in the performance index
% tsym:   time
%         (symbolic representation)
% d:      approximation degree
%         The highest degree of monomials for \hat{u} is
%         d.
%         The highest degree of monomials for \hat{V} is
%         d+1.
% adpOpt: settings of the function adpModelBased
% ========================================================
% ======================== Output ========================
% w:      w in \hat{u} = w * Phi_u
% c:      c in \hat{V} = Phi_V * c
% adpOpt: settings of the function adpModelBased
%         In case that default options are used, the user
%         may output this term to check details.
% ========================================================

switch nargin
    case 10
        adpOpt = adpSetModelBased();
        fprintf('Working...\n\n');
    case 11
        fprintf('Working...\n\n');
    otherwise
        fprintf('Please check inputs of the function ');
        fprintf('adpModelBased.\n');
end

if isempty(adpOpt.xInit) % use default initial states
    xInit = defInitStat(n, adpOpt.xInitNum, adpOpt.xInitMin,...
        adpOpt.xInitMax);
    adpOpt.xInit = xInit;
else
    xInit = adpOpt.xInit;
    adpOpt.xInitNum = size(xInit,1);
end
tSpan = adpOpt.tSpan;

if isempty(adpOpt.explSymb) % use default exploration noise
    esym = defNoise(m, tsym, adpOpt.explAmpl, adpOpt.numFreq,...
        adpOpt.xInitNum);
    adpOpt.explSymb = esym;
else
    if size(adpOpt.explSymb, 1) < (adpOpt.xInitNum * m)
        fprintf('Error: There are not enough exploration');
        fprintf(' signals');
    else
        esym = adpOpt.explSymb;
    end
end

if isempty(adpOpt.u0Symb) % use default initial control
    u0sym = defInitCtrl(fsym, gsym, xsym, usym, qsym, n, m, R);
    adpOpt.u0Symb = u0sym;
else
    if size(adpOpt.u0Symb, 1) ~= m
        fprintf('Error: The dimension of the initial control');
        fprintf(' is wrong.');
    else
        u0sym = adpOpt.u0Symb;
    end
end

% Define somw basic parameters.
stride  = adpOpt.stride;
iterMax = adpOpt.maxIter;
crit    = adpOpt.crit;
epsilon = adpOpt.epsilon;
opt     = adpOpt.opt;
odeOpt  = adpOpt.odeOpt;

% Number of basis functions for \hat{V} and \hat{u}.
[N1, N2] = basisNum(n, d, opt);

% Convert symbolic functions to matlab functions.
[sys, u0, q] = sf2mf(fsym, gsym, xsym, usym, u0sym, qsym);

% Generate data.
[data, N] = dataGeneModelBased(sys, u0, esym, tsym, q,...
    xInit, tSpan, m, N2, R, opt, odeOpt);

[Phi_V, Costx, Costu0, alpha, beta, gamma] = dataProc(data,...
    N, stride, n, N1, m, N2, opt);

% ADP iterations.
[w, c] = adpIter(iterMax, N1, N2, R, Phi_V, Costx, Costu0,...
    alpha, beta, gamma, epsilon, crit);

% Create 2 .m files: uAdp.m and VAdp.m.
create_uAdp(n, m, N2, w);
create_VAdp(n, N1, c);

% Symbolic form of the approximate optimal control.
% To be compatible with earlier MATLAB versions, variables
% x_u_extend and x_v_extend are created.
x = xsym;
assume(x, 'real');

x_u_extend = ones(N2,1) * x';
Du = basisMono(n,N2);
uAdpSym = w * prod(x_u_extend .^ Du, 2);
uAdpSymDec = vpa(uAdpSym, 4);
% Symbolic form of the approximate optimal cost function.
DV = basisMono(n,N1);
x_v_extend = ones(N1,1) * x';
VAdpSym = c' * prod(x_v_extend.^DV, 2);
VAdpSymDec = vpa(VAdpSym, 4);

% uAdp = matlabFunction(uAdpSym, 'Vars', {x});
% save('uAdp.mat', 'uAdp');

fprintf('The approximate optimal control u(x) ');
fprintf('(shown with 4 significant digits):\n');
fprintf('\n');
disp(uAdpSymDec);
fprintf('The approximate optimal cost function V(x) ');
fprintf('(shown with 4 significant digits):\n\n');
disp(VAdpSymDec);

fprintf('The files uAdp.m and VAdp.m have been generated.\n\n');
fprintf('The approximate optimal control can be applied by ');
fprintf('calling\n');
fprintf('>> u = uAdp(x)\n');
fprintf('where x is the state vector.\n\n');
fprintf('Similarly, the approximate optimal cost function can ');
fprintf('be applied by calling\n');
fprintf('>> V = VAdp(x)\n\n');

% Create a .txt file.
create_txt(m, uAdpSym, uAdpSymDec, VAdpSym, VAdpSymDec);

% c = c'; % to be consistent with the paper

end

function create_uAdp(n, m, N2, w)
% >> This function creates a .m file and save the approximate
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
% >> This function creates a .m file and save the approximate
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

fprintf(fid, '%s', 'The files uAdp.m and VAdp.m have been');
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
