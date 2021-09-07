function dX = sysAdp(~,X)
% System dynamics + cumulative cost.

I = diag([0.1029, 0.1263, 0.0292]);
e = [1;0;0;0;0;0;0]; % equalibrium point

Q = 2*eye(7);
R = eye(3);

x = X(1:7);
u = uAdp(x-e);

qua = X(1:4); % quaternion
ome = X(5:7); % omega

dqua = quaternion_prod(qua,ome)/2;
% dqua = quaternion_prod(qua,ome)/2 - alpha*(qua'*qua-1)*qua;
dome = pinv(I) * (cross(I*ome,ome) + u);

dcost = (x-e)'*Q*(x-e) + u'*R*u;

dX = [dqua; dome; dcost];
end