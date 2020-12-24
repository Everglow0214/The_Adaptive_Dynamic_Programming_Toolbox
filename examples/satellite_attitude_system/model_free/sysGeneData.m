function dx = sysGeneData(t,x,K,i)
% System dynamics.

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