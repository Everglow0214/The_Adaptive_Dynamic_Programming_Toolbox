function dX = sysAdp(~,X)
% To be used in the function 'ode45'.
% Parameters of the system.
k1 = 3;
k2 = 2;
b = 2;
mass = 5;
% For the cost function.
Q = [5, 0;
     0, 3];
r = 2;

x1 = X(1); % state x1
x2 = X(2); % state x2
x = [x1; x2];
u = uAdp(x); % call the approximate control

% System dynamics.
dx1 = x2;
dx2 = (-k1*x1-k2*x1^3-b*x2+u)/mass;
% To calculate the integral costs.
dcost = x'*Q*x + r*u*u;
dX = [dx1; dx2; dcost];
end