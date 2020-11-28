%% Main part.
clear; clc; close all;
tic

BEGIN_ACADO;
    acadoSet('problemname', 'main');
    
    % States
    DifferentialState q1 q2 q3 q4; % err = q - 1
    DifferentialState o1 o2 o3;
    % Control
    Control tau1 tau2 tau3;
    
    % Some parameters
    alpha = 1;
    J1 = 0.1029;
    J2 = 0.1263;
    J3 = 0.0292;
    
    % Initial state.
    theta = 1.99999 * pi;
    qo_init = [cos(theta/2); sin(theta/2); 0; 0; 0; 0; 0];
    
    % System dynamics.
    f = acado.DifferentialEquation();
    f.add(dot(q1) == 0.5*(-q2*o1 - q3*o2 - q4*o3) -...
        alpha*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1)*q1);
    f.add(dot(q2) == 0.5*(q1*o1 - q4*o2 + q3*o3) -...
        alpha*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1)*q2);
    f.add(dot(q3) == 0.5*(q4*o1 + q1*o2 - q2*o3) -...
        alpha*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1)*q3);
    f.add(dot(q4) == 0.5*(-q3*o1 + q2*o2 + q1*o3) -...
        alpha*(q1*q1 + q2*q2 + q3*q3 + q4*q4 - 1)*q4);
    f.add(dot(o1) == ((J2-J3)*o2*o3+tau1) / J1);
    f.add(dot(o2) == ((J3-J1)*o1*o3+tau2) / J2);
    f.add(dot(o3) == ((J1-J2)*o1*o2+tau3) / J3);
    
    % Optimal control problem
    ocp = acado.OCP(0.0, 30.0, 600);
    
    % For LSQ
    S = [eye(7)*2, zeros(7,3);
         zeros(3,7), eye(3)];
    h = {q1, q2, q3, q4, o1, o2, o3, tau1, tau2, tau3};
    r = zeros(1,10);
    r(1) = 1;
    ocp.minimizeLSQ(S, h, r);
    
    ocp.subjectTo(f);
    ocp.subjectTo('AT_START', q1 == qo_init(1));
    ocp.subjectTo('AT_START', q2 == qo_init(2));
    ocp.subjectTo('AT_START', q3 == qo_init(3));
    ocp.subjectTo('AT_START', q4 == qo_init(4));
    ocp.subjectTo('AT_START', o1 == qo_init(5));
    ocp.subjectTo('AT_START', o2 == qo_init(6));
    ocp.subjectTo('AT_START', o3 == qo_init(7));
    
    % Optimization algorithm
    algo = acado.OptimizationAlgorithm(ocp);
END_ACADO

out = main_RUN();

running_time = toc;

%% Simulation with the controller obtained.
u = out.CONTROLS;

X = [qo_init; 0]';

X_save = X;
t_save = 0;

T = 0.05;

odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

for i = 1:(size(u,1)-1)
   [t,X] = ode45(@(t,X) sys(t,X,u(i,2:4)'), [0,T],...
                 X(end,:),odeOpt);
   t_save = [t_save; i*T];
   X_save = [X_save; X(end,:)];
end

e = [1,0,0,0,0,0,0]; % equalibrium point

figure(1);
plot(t_save, sqrt(sum((X_save(:,1:7)-e).^2,2)));
grid on;
xlabel('$t$', 'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman');
ylabel('$\Vert x(t)\Vert$',...
       'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman');
   
figure(2);
plot(t_save, X_save(:,end));
grid on;
xlabel('$t$', 'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman');
ylabel('Cost',...
       'Interpreter', 'Latex',...
       'Fontname', 'Times New Roman');

%% Local functions.
% System dynamics + cumulative cost.
function dX = sys(~,X,u)
alpha = 1;
I = diag([0.1029, 0.1263, 0.0292]);
e = [1;0;0;0;0;0;0]; % equalibrium point

Q = 2*eye(7);
R = eye(3);

x = X(1:7);

qua = X(1:4); % quaternion
ome = X(5:7); % omega

dqua = quaternion_prod(qua,ome)/2 - alpha*(qua'*qua-1)*qua;
dome = pinv(I) * (cross(I*ome,ome) + u);

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