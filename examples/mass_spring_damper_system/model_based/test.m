%% Test the approximate control.
stateInit = [-2, 2.5]; % initial state
timeSpan = [0,30];
odeOpt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);

% Calculate the trajectory under the approximate control.
% '0' is added to calculate the integral costs.
[tAdp, xAdp] = ode45(@(t,X) sysAdp(t,X), timeSpan,...
                     [stateInit, 0], odeOpt);

% Plot results.
figure(1);
plot(tAdp, xAdp(:,1));
hold on;
plot(tAdp, xAdp(:,2));
grid on;
legend({'$x_1$', '$x_2$'}, 'Interpreter', 'Latex');
xlabel('Time $t$ [s]', 'Interpreter', 'Latex');
title('Trajectories');

figure(2);
plot(tAdp, xAdp(:,3));
grid on;
xlabel('Time $t$ [s]', 'Interpreter', 'Latex');
title('Integral costs');

%% Starting in MATLAB R2016b, scripts can contain codes
%  to define functions. Thus if you are using MATLAB R2016b or
%  or later versions, you can uncomment the following codes.
%% To be used in the function 'ode45'.
% function dX = sysAdp(~,X)
% % Parameters of the system.
% k1 = 3;
% k2 = 2;
% b = 2;
% mass = 5;
% % For the cost function.
% Q = [5, 0;
%      0, 3];
% r = 2;
% 
% x1 = X(1); % state x1
% x2 = X(2); % state x2
% x = [x1; x2];
% u = uAdp(x); % call the approximate control
% 
% % System dynamics.
% dx1 = x2;
% dx2 = (-k1*x1-k2*x1^3-b*x2+u)/mass;
% % To calculate the integral costs.
% dcost = x'*Q*x + r*u*u;
% dX = [dx1; dx2; dcost];
% end