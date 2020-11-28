%% Test the approximate control.
theta = 1.99999*pi;
x0 = [cos(theta/2), sin(theta/2), 0, 0, 0, 0, 0];
tSpan = [0, 40];
odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% The control by ADPT.
[t,X] = ode45(@(t,X) sys(t,X), tSpan, [x0,0], odeOpt);

e = [1,0,0,0,0,0,0]; % equalibrium point

figure(1);
plot(t, sqrt(sum((X(:,1:7)-e).^2,2)));
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
       'Fontname', 'Times New Roman');

%% Local functions.
% System dynamics + cumulative cost.
function dX = sys(~,X)
alpha = 1;
I = diag([0.1029, 0.1263, 0.0292]);
e = [1;0;0;0;0;0;0]; % equalibrium point

Q = 2*eye(7);
R = eye(3);

x = X(1:7);
u = uAdp(x-e);

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