% LQR section
Q  = [1 0 0; 0 1 0; 0 0 1];
Qf = eye(3);
R  = eye(2)*10^-5;

x0 = [0 0 0]';

Radius = 0.02;
L = 0.189;
%% Simulation
t_end=90;                               % Max time of simulation [s]

%Fixed step, 0.1, Euler
out=sim('SimpleLQR', t_end);
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
