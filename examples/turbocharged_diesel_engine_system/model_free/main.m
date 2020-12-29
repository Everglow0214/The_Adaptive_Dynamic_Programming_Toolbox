clear; clc; close all;

%% Generate data.
x = sym('x', [6,1]);
t = sym('t');

% If one uses the following way to define a symbolic variable, an
% error would occur if the MATLAB version is earlier than 2019a.
% syms x [6,1] real
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
% System dynamics (\dot{x} = f(x) + g(x)*u).
f = A * x;
g = B;

% Feedback gain. The initial control is u0 = -K*x.
K = ones(2,6);

% Exploration signals.
e1 = 0.8*(sin(7*t) + sin(1.1*t) + sin(sqrt(3)*t) + sin(sqrt(6)*t));
e2 = 0.8*(sin(2*t) + sin(2*pi*t)) + sin(sqrt(10)*t) + sin(11*t);
noiseSym = [e1, e2];
noise = matlabFunction(noiseSym, 'Vars', t);

% To be used in the function ode45.
dx = matlabFunction(f+g*(-K*x+noiseSym'), 'Vars', {t,x});

xInit = -3 + 6 * rand(4,6);
tSpan = ones(4,1) * [0:0.002:8];
odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

t_save = []; % to record timestamps
x_save = []; % to record values of states at the timestamps in t_save
for i = 1:size(xInit,1)
    [time,states] = ode45(@(t,x) dx(t,x), tSpan(i,:),...
                          xInit(i,:), odeOpt);
    t_save = [t_save; time];
    x_save = [x_save; states];
end

% To record values of initial controls at the timestamps in 't_save'.
u0 = -x_save * K';
% To record values of exploration signals at the timestamps in 't_save'.
eta = noise(t_save);

%% Main part.
% Approximation degree.
% For the approximate optimal control, the degree is up to d.
% For the approximate optimal cost function, the degree is up to d+1.
d = 1;

%% Some basic information of the system.
n = 6; % state dimension
m = 2; % control dimension

%% Define the performance index.
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.
Q = eye(n);
Q(3,3) = 0.1;
Q(4,4) = 0.1;
Q(5,5) = 0.1;
Q(6,6) = 0.1;
q = @(x) x'*Q*x;
R = eye(m);


%% Specify settings.
stride = 3;
epsilon = 0.0001;
adpOpt = adpSetModelFree('stride', stride, 'epsilon', epsilon);

%% Execute ADP iterations.
if exist('adpOpt', 'var') % specified settings would be applied
    [w,c] = adpModelFree(t_save,x_save,n,u0,m,eta,d,q,R,adpOpt);
else % default settings would be applied
    [w,c] = adpModelFree(t_save,x_save,n,u0,m,eta,d,q,R);
end