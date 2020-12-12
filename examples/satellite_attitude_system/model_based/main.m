%%
clear; clc; close all;
%%
tic

% Approximation degree.
% For the approximate optimal control, the degree is up to d.
% For the approximate optimal cost function, the degree is up to d+1.
d = 2;

%% Define the system.
% The system is given by
% \dot{x} = f(x) + g(x)*u.

n = 7; % state dimension
m = 3; % control dimension

syms x [n,1] real
syms u [m,1] real
syms t real

alpha = 1;
I = diag([0.1029, 0.1263, 0.0292]);
qe = [1;0;0;0];

f1 = quaternion_prod(x(1:4)+qe,x(5:7))/2 -...
    alpha*((x(1:4)+qe)'*(x(1:4)+qe)-1)*(x(1:4)+qe);
f2 = pinv(I)*cross(I*x(5:7),x(5:7));
f = [f1;
     f2];
g = [zeros(4,3);
     pinv(I)];

%% Define the performance index.
% The performance index is given by
% \int_{0}^{\infty} q(x) + u'*R*u dt.

Q = eye(n)*2;
q = x'*Q*x;
R = eye(m);

%% Specify settings of ADP.
xInit = [-0.2,  0.4,  0.2, -0.4,  0.1, -0.1, -0.05;
         -0.6, -0.8, -0.2, -0.4,  0.1, -0.1, -0.15;
          0.4,  0.8,  0.4, -0.2, -0.2, 0.15,   0.2];

tSpan = ones(3,1)*[0:0.01:15];

e1 = 0.72*(sin(3*pi*t) + sin(7*t) + sin(11*t) +...
          sin(15*t) + sin(17*t) + sin(sqrt(11)*t));
e2 = 0.7*(sin(sqrt(2)*t) + sin(3*t) + sin(7*t) +...
          sin(11*t) + sin(13*t) + sin(15*t));
e3 = 0.72*(sin(2*t) + sin(7*pi*t) + sin(sqrt(6)*t) +...
          sin(sqrt(10)*t) + sin(11*pi*t) + sin(13*t));
      
e4 = 0.7*(sin(3*pi*t) + sin(7*t) + sin(11*t) +...
          sin(15*t) + sin(17*t) + sin(sqrt(11)*t));
e5 = 0.66*(sin(sqrt(2)*t) + sin(3*t) + sin(7*t) +...
          sin(11*t) + sin(13*t) + sin(15*t));
e6 = 0.68*(sin(2*t) + sin(7*pi*t) + sin(sqrt(6)*t) +...
          sin(sqrt(10)*t) + sin(11*pi*t) + sin(13*t));
      
e7 = 0.68*(sin(3*pi*t) + sin(7*t) + sin(11*t) +...
          sin(15*t) + sin(17*t) + sin(sqrt(11)*t));
e8 = 0.72*(sin(sqrt(2)*t) + sin(3*t) + sin(7*t) +...
          sin(11*t) + sin(13*t) + sin(15*t));
e9 = 0.7*(sin(2*t) + sin(7*pi*t) + sin(sqrt(6)*t) +...
          sin(sqrt(10)*t) + sin(11*pi*t) + sin(13*t));

e = [e1;e2;e3;e4;e5;e6;e7;e8;e9];

adpOpt = adpSetModelBased('explSymb', e,...
                          'xInit', xInit,...
                          'tSpan', tSpan);

%% Execute ADP approximations.
[w,c,adpOpt] = adpModelBased(f,g,x,n,u,m,q,R,t,d,adpOpt);

running_time = toc;

%% Local functions.
% Quaternion product.
function p = quaternion_prod(q,r)
if numel(r) == 3
    r = [0; r];
end
q_matrix = [q(1) -q(2) -q(3) -q(4);
            q(2) q(1) -q(4) q(3);
            q(3) q(4) q(1) -q(2);
            q(4) -q(3) q(2) q(1)];
p = q_matrix * r;
end