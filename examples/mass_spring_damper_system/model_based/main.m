clear; clc; close all;

%% Approximation degree.
% For the approximate optimal control, the degree is up to d.
% For the approximate optimal cost function, the degree is up to d+1.
d = 3;

%% Define the system.
% The system is given by
% \dot{x} = f(x) + g(x)*u.

n = 2; % state dimension
m = 1; % control dimension
syms x [n,1] real
syms u [m,1] real
syms t real

% Parameters of the system.
k1 = 3;
k2 = 2;
b = 2;
mass = 5;

% Define f(x) and g(x).
f = [x2;
     (-k1*x1-k2*x1^3-b*x2)/mass];
g = [0;
     1/mass];

%% Define the performance index.
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.
Q = eye(n);
Q(1,1) = 5;
Q(2,2) = 3;
q = x'*Q*x;
R = eye(m);
R(1,1) = 2;

%% Specify settings.
xInit = [-3, 2;
         2.2, 3;
         -2, -2];
tSpan = [0, 10;
         0, 10;
         0, 10];
adpOpt = adpSetModelBased('xInit', xInit, 'tSpan', tSpan);

%% Execute ADP iterations.
if exist('adpOpt','var') == 1 % specified settings would be used
    [w,c,adpOpt] = adpModelBased(f,g,x,n,u,m,q,R,t,d,adpOpt);
else % default setting would be applied
    [w,c,adpOpt] = adpModelBased(f,g,x,n,u,m,q,R,t,d);
end