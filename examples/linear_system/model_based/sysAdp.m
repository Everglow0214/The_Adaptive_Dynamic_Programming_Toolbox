function dX = sysAdp(~,X)
% To be used in the function 'ode45'.

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
       
% For the cost function.
Q = eye(6);
Q(3,3) = 0.1;
Q(4,4) = 0.1;
Q(5,5) = 0.1;
Q(6,6) = 0.1;
R = eye(2);

x = X(1:6);
u = uAdp(x); % call the approximate control
% K = lqr(A,B,Q,R);
% u = -K * x;

% System dynamics.
dx = A*x + B*u;
% To calculate the integral costs.
dcost = x'*Q*x + u'*R*u;
dX = [dx; dcost];
end