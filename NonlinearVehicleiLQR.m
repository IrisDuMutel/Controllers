% LQR section
Q  = eye(3)*10;
Qf = eye(3);
R  = eye(2)*10^-2;
Radius = 0.02;
L = 0.189;
dt = 0.01;

toll = 3*10^-5;
x0 = [0 0 0]';

% A = [1 0 dt*-sin(psi)*Radius/2*(wR+wL);...
%      0 1 dt*cos(psi)*Radius/2*(wR+wL);...
%      0 0 1];
% L = dt*[cos(psi)*Radius/2 cos(psi)*Radius/2;...
%      sin(psi)*Radius/2 sin(psi)*Radius/2;...
%      -Radius/L Radius/L];

%% Simulation
t_end=90;                               % Max time of simulation [s]

%Fixed step, 0.1, Euler
out=sim('NonLinearVehicleiLQR', t_end);
clc

%%
ts=out.time;
Xs=out.x;
Ys=out.y;
% VXs=out.Xdot.signals.values(:,1);
% VYs=out.Ydot.signals.values(:,1);
psis=out.theta;
pwm_l=out.w(:,1);
pwm_r=out.w(:,2);
% Vx_ref=out.Vx_ref.signals.values(:,1);
% psi_ref=out.psi_ref.signals.values(:,1);
% psidots=out.psidot.signals.values(:,1);
% Vx=out.Vx.signals.values(:,1);

plot(Xs)
