clear; clc; close all;

%% Approximation degree.
% For the approximate optimal control, the degree is up to d.
% For the approximate optimal cost function, the degree is up to d+1.
d = 1;

%% Define the system.
% The system is given by
% \dot{x} = f(x) + g(x)*u.

n = 6; % state dimension
m = 2; % control dimension
x = sym('x', [n,1]);
u = sym('u', [m,1]);
t = sym('t');

% If one uses the following way to define a symbolic variable, an
% error would occur if the MATLAB version is earlier than R2019a.
% syms x [n,1] real
% syms u [m,1] real
% syms t real

% Parameters of the system.
A = [ -0.4125, -0.0248,  0.0741,    0.0089,        0,        0;
     101.5873, -7.2651,  2.7608,    2.8068,        0,        0;
       0.0704,  0.0085, -0.0741,   -0.0089,        0,   0.0200;
       0.0878,  0.2672,       0,   -0.3674,   0.0044,   0.3962;
      -1.8414,  0.0990,       0,         0,  -0.0343,  -0.0330;
            0,       0,       0, -359.0000, 187.5364, -87.0316];
B = [-0.0042,  0.0064;
     -1.0360,  1.5849;
      0.0042,       0;
      0.1261,       0;
           0, -0.0168;
           0,       0];

% Define f(x) and g(x).
f = A * x;
g = B;

%% Define the performance index.
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.
Q = eye(n);
Q(3,3) = 0.1;
Q(4,4) = 0.1;
Q(5,5) = 0.1;
Q(6,6) = 0.1;
q = x'*Q*x;
R = eye(m);

%% Specify settings.
xInit = -3 + 6 * rand(4,6); % choose initial states randomly
tSpan = ones(4,1) * [0, 10];
adpOpt = adpSetModelBased('xInit', xInit, 'tSpan', tSpan);

%% Execute ADP iterations.
if exist('adpOpt','var') == 1 % specified settings would be used
    [w,c,adpOpt] = adpModelBased(f,g,x,n,u,m,q,R,t,d,adpOpt);
else % default setting would be applied
    [w,c,adpOpt] = adpModelBased(f,g,x,n,u,m,q,R,t,d);
end