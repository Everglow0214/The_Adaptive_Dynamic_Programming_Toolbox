%%
clear; clc; close all;

%% Generate data.

xInit = [-0.2+1,  0.4,  0.2, -0.4,  0.1, -0.1, -0.05;
         -0.6+1, -0.8, -0.2, -0.4,  0.1, -0.1, -0.15;
          0.4+1,  0.8,  0.4, -0.2, -0.2, 0.15,   0.2;
         -0.8+1,  0.4,  0.8,  0.4, 0.14, 0.32, -0.25];

tSpan = ones(4,1)*[0:0.002:15];

odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% The initial control is chosen as a linear state feedback.
A = [-2, zeros(1,6);
     zeros(3,4), eye(3)/2;
     zeros(3,7)];
I = diag([0.1029, 0.1263, 0.0292]);
B = [zeros(4,3);
     pinv(I)];
Q = eye(7)*2;
R = eye(3);
K = lqr(A,B,Q,R);

t_save = [];
x_save = [];
eta_save = [];
for i = 1:size(xInit,1)
    [time, states] = ode45(@(t,x) sys(t,x,K,i),...
                           tSpan(i,:),...
                           xInit(i,:),...
                           odeOpt);
    t_save = [t_save; time];
    x_save = [x_save; states-[1,0,0,0,0,0,0]];
    eta_save = [eta_save; noise(time,i)]; % exploration signals
end

u0_save = -(x_save-[1,0,0,0,0,0,0]) * K'; % initial controls

%% Main part.
tic

% Approximation degree.
% For the approximate optimal control, the degree is up to d.
% For the approximate optimal cost function, the degree is up to d+1.
d = 2;

n = 7; % state dimension
m = 3; % control dimension

%% Define the performance index.
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.
Q = eye(n)*2;
q = @(x) x'*Q*x;
R = eye(m);

%% Specify settings of ADP.
adpOpt = adpSetModelFree('stride', 4);

%% Execute ADP approximations.
[w,c,adpOpt] = adpModelFree(t_save,x_save,n,u0_save,m,eta_save,d,q,R,adpOpt);

running_time = toc;

%% Local functions.
% System dynamics.
function dx = sys(t,x,K,i)
alpha = 1;
I = diag([0.1029, 0.1263, 0.0292]);
e = [1;0;0;0;0;0;0]; % equalibrium point
u = -K*(x-e) + noise(t,i)';

qua = x(1:4); % quaternion
ome = x(5:7); % omega

dqua = quaternion_prod(qua,ome)/2 - alpha*(qua'*qua-1)*qua;
dome = pinv(I) * (cross(I*ome,ome) + u);
dx = [dqua; dome];
end

% Quaternion product.
function p = quaternion_prod(q,r)
if numel(r) == 3
    r = [0;r];
end
q_matrix = [q(1) -q(2) -q(3) -q(4);
            q(2) q(1) -q(4) q(3);
            q(3) q(4) q(1) -q(2);
            q(4) -q(3) q(2) q(1)];
p = q_matrix * r;
end

% Exploration signals.
function e = noise(t,i)
ampl = [0.72,  0.7, 0.72;
          0.7, 0.66, 0.68;
         0.68, 0.72,  0.7;
         0.57, 0.58, 0.69];
     
e1 = ampl(i,1)*(sin(3*pi*t) + sin(7*t) + sin(11*t) +...
     sin(15*t) + sin(17*t) + sin(sqrt(11)*t));
 
e2 = ampl(i,2)*(sin(sqrt(2)*t) + sin(3*t) + sin(7*t) +...
     sin(11*t) + sin(13*t) + sin(15*t));
 
e3 = ampl(i,3)*(sin(2*t) + sin(7*pi*t) + sin(sqrt(6)*t) +...
     sin(sqrt(10)*t) + sin(11*pi*t) + sin(13*t));
 
e = [e1, e2, e3];
end