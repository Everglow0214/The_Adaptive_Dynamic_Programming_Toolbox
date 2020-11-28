%%
clear; clc; close all;
%%
tic

% Degrees of monomials for the control are up to d.
% Degrees of monomials for the cost function are up to d+1.
d = 3;

%% Define the system.
n = 7; % state dimension
m = 3; % control dimension

x = sym('x',[n,1]); % state variables, eq and eomega
u = sym('u',[m,1]); % control variables

J1 = 0.1029;
J2 = 0.1263;
J3 = 0.0292;
I = diag([J1,J2,J3]); % inertial matrix

alpha = 1;

xd = [0;0;0;0;0;0;0]; % equilibrium state of error dynamics
ud = [0;0;0]; % equilibrium control

% Desired q and omega
qe = [1;0;0;0];
oe = [0;0;0];
qoe = [qe; oe];

% Error dynamics
% eq = q - qe;
% eo = o - eo;
% eqdot = (eq+qe)*eomega/2 - alpha*(|eq+qe|^2-1)*(eq+qe)
% eodot = inv(I) * ((I*eomega) X eomega + u)
feq = quat_product(x(1:4,1)+qe, x(5:7,1))/2 -...
    alpha*(((x(1:4,1)+qe)')*(x(1:4,1)+qe)-1)*(x(1:4,1)+qe);
feo = pinv(I)*(cross((I*x(5:7,1)), x(5:7,1)) + u(1:3,1));
fsym = [feq;feo];

%% Define the performance index.
% \int_{0}^{\infty} (x'*Q*x + u'*R*u) dt
Q = 2*eye(n);
R = eye(3);
qsym = x'*Q*x + u'*R*u;

xscale=ones(n,1);
uscale=ones(m,1);

f_sym=sym([]);
n_=0;
x_sym=sym([]);
x_0=[];
x_scale=[];

%% Main part.
% Call 'hjb_set_up.m' to convert the symbolic fsym and lsym into
% the matrices f and l of their Taylor polynomials at xd and ud.
[f,q] = hjb_set_up(fsym, qsym, x, u, xd, ud, xscale, uscale,...
    n, m, d, f_sym, n_, x_sym, x_0, x_scale);

% Call 'hjb.m' to find the Taylor polynomial cop of the optimal
% cost to degree d+1 and the Taylor polynomial uop of the optimal
% feedback control to degree d.
% tic;
[uop, fk, cop, ck] = hjb(f, q, n, m, d);

running_time = toc;

%% Simulation with the controller obtained.
cop_fn = @(xx) cop*mon((xx-xd)./xscale, n, [2,d+1]);
uop_fn = @(xx) uop*mon((xx-xd)./xscale, n, [1,d]);

theta = 1.99999 * pi;
x0 = [cos(theta/2), sin(theta/2), 0, 0, 0, 0, 0];
tSpan = [0, 40];
odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% The control by NST.
[t,X] = ode45(@(t,X) sys(t,X,uop_fn), tSpan, [x0,0], odeOpt);

figure(1);
plot(t, sqrt(sum((X(:,1:7)-qoe').^2,2)));
grid on;
xlabel('$t$', 'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman');
ylabel('$\Vert x(t)\Vert$',...
       'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman');
   
figure(2);
plot(t, X(:,end));
grid on;
xlabel('$t$', 'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman');
ylabel('Cost',...
       'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman')

%% Local functions.
% System dynamics + cumulative cost.
function dX = sys(~,X,uop_fn)
alpha = 1;
I = diag([0.1029, 0.1263, 0.0292]);
e = [1;0;0;0;0;0;0]; % equalibrium point

Q = 2*eye(7);
R = eye(3);

x = X(1:7);
u = uop_fn(x-e);

qua = X(1:4); % quaternion
ome = X(5:7); % omega

dqua = quaternion_prod(qua,ome)/2 - alpha*(qua'*qua-1)*qua;
dome = pinv(I)*(cross(I*ome,ome) + u);

dcost = (x-e)'*Q*(x-e) + u'*R*u;

dX = [dqua; dome; dcost];
end

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