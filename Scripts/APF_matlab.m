%% Simulation value
clear , clc
%Load funzione DCmotor
load('Model_nlarx.mat');

t_end=60;
prompt = {'X_{goal}','Y_{goal}','X_0','Y_0','psi_0 in deg',' X_{obs}','Y_{obs}'};
dlgtitle = 'Welcome to Devastator Simulator';
dims = [1 17; 1 17; 1 17; 1 17; 1 17; 1 17; 1 17];
options.Resize='on';
options.WindowStyle='normal';
options.Interpreter='tex';
x = inputdlg(prompt,dlgtitle,dims, {'' '' '' '' '' '' ''},options);
 
X_goal= str2num(x{1});
Y_goal= str2num(x{2});
X_0= str2num(x{3});
Y_0= str2num(x{4});
psi_0= str2num(x{5})/180*pi;
ox= str2num(x{6});
oy= str2num(x{7});

% X_goal=1;           % Desired X position  [m]
% Y_goal=0;           % Desired Y position  [m]
% X_0=0;              % Initial X position  [m]
% Y_0=0;              % Initial Y position  [m]
% ox=5;               % Obstacle X position [m]
% oy=5;               % Obstacle Y position [m]
Vx_max=0.3;        % Max speed           [m/s]
robot_radius = 0.3; % Robot dimensions    [m]
%%
% PID section
 Kp_Vx=2;%2.7;
 Kd_Vx=0;
 Ki_Vx=1.85;%0.5;
 Kp_psi=0.3;%0.7;
 Kd_psi=0;
 Ki_psi=0;

 N=5;

%%
t_end=60;
out=sim('Modello_completo', t_end);
clc

%% Results

ts=out.tout;
Xs=out.X.signals.values(:,1);
Ys=out.Y.signals.values(:,1);
Vxs=out.Xdot.signals.values(:,1);
Vys=out.Ydot.signals.values(:,1);
psis=out.psi.signals.values(:,1);
psidots=out.psidot.signals.values(:,1);

%   anime_plot_2(Xs, Ys, X_goal, Y_goal, ox, oy, robot_radius, influence_area)
%%
figure()
plot(Xs,Ys)
grid on
hold on
xlabel('X [m]')
ylabel('Y [m]')
title('Position')
plot(X_goal,Y_goal,'or')
plot(ox,oy,'ok')
ang=0:0.01:2*pi;
xr=0.5*cos(ang);
yr=0.5*sin(ang);
for i=1:length(ox)
plot(xr+ox(i),yr+oy(i),'--m')
end
xrr=robot_radius*cos(ang);
yrr=robot_radius*sin(ang);
plot(xrr+Xs(end),yrr+Ys(end),'--c')
axis equal

%%
figure()
plot(ts,Xs)
grid on
hold on
title('X position')
xlabel('Time [s]')


figure()
plot(ts,Ys)
title('Y position')
xlabel('Time [s]')
ylabel('Y [m]')
grid on

%%
figure()
plot(ts,Vxs)
grid on
hold on
plot(ts,Vys)
xlabel('Time [s]')
ylabel('V [m/s]')
sgtitle('Speed')

figure()
plot(ts,psidots)
grid on
xlabel('Time [s]')
ylabel('\psidot [rad/s]')
sgtitle('Angular speed')
figure()
plot(ts,psis*180/pi)
grid on
xlabel('Time [s]')
ylabel('\psi [deg]')
sgtitle('Angular position')

%% Sezione solo APF

X_goal=1;           % Desired X position  [m]
Y_goal=0;           % Desired Y position  [m]
X_0=0;              % Initial X position  [m]
Y_0=0;              % Initial Y position  [m]
ox=5;               % Obstacle X position [m]
oy=5;               % Obstacle Y position [m]
Vx_max=0.45;        % Max speed           [m/s]
robot_radius = 0.3; % Robot dimensions    [m]

% PID section
 Kp_Vx=3;%2.7;
 Kd_Vx=0;
 Ki_Vx=1.5;%0.5;
 Kp_psi=1.5;%1.75;
 Kd_psi=0;
 Ki_psi=0;
 
%%
% setenv('ROS_MASTER_URI','http://192.168.79.128:11311')
% setenv('ROS_IP','192.168.79.128')
% rosinit
