clear; clc; close all;

%% Generate data.
x = sym('x', [2,1]);
t = sym('t');
x1 = x(1);
x2 = x(2);

% If one uses the following way to define a symbolic variable, an
% error would occur if the MATLAB version is earlier than R2019a.
% syms x [2,1] real
% syms t real

% Parameters of the system.
k1 = 3;
k2 = 2;
b = 2;
mass = 5;
% System dynamics.
f = [x2;
     (-k1*x1-k2*x1^3-b*x2)/mass];
g = [0;
     1/mass];

% Feedback gain. The initial control is u0 = -K*x.
K = [1,1];

% Exploration signals.
noiseSym = 0.8*(sin(7*t) + sin(1.1*t) + sin(sqrt(3)*t) + sin(sqrt(6)*t));
noise = matlabFunction(noiseSym, 'Vars', t);

% To be used in the function ode45.
dx = matlabFunction(f+g*(-K*x+noiseSym), 'Vars', {t,x});

xInit = [-3, 2;
         2.2, 3;
         -2, -2];
tSpan = [0:0.002:8;
         0:0.002:8;
         0:0.002:8];
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
d = 3;

%% Some basic information of the system.
n = 2; % state dimension
m = 1; % control dimension

%% Define the performance index.
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.
Q = [5, 0;
     0, 3];
q = @(x) x'*Q*x;
R = 2;


%% Specify settings.
stride = 3;
adpOpt = adpSetModelFree('stride', stride);

%% Execute ADP iterations.
if exist('adpOpt', 'var') % specified settings would be applied
    [w,c] = adpModelFree(t_save,x_save,n,u0,m,eta,d,q,R,adpOpt);
else % default settings would be applied
    [w,c] = adpModelFree(t_save,x_save,n,u0,m,eta,d,q,R);
end