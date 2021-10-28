%% Simulation Model
clear all, clc, close all

%Load modello del robot
load('Model_nlarx.mat');

%Inserimento dei parametri per la "missione"
prompt = {'X_{goal}','Y_{goal}','X_0','Y_0','psi_0 in deg(-180,180)',' X_{obs}','Y_{obs}'};
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
% psi_0=0;            % Initial orientation [rad]
% ox=5;               % Obstacle X position [m]
% oy=5;               % Obstacle Y position [m]

%% Controller
% PID section
 Kp_Vx=2; 
 Kd_Vx=0;
 Ki_Vx=1.85;
 Kp_psi=0.4;
 Kd_psi=0;
 Ki_psi=0;
 
 % LQR section
Q  = 1*[0.1 0; 0 0.01];
Qf = 1*[1 0; 0 1 ];
R  = 1*[1e-2 0; 0 1];
r = 0.02;
b = 0.189;
dt = 0.001;

%% Simulation
t_end=90;                               % Max time of simulation [s]

%Fixed step, 0.1, Euler
out=sim('Devastator_simulator', t_end);
clc
%%
ts=out.tout;
Xs=out.X.signals.values(:,1);
Ys=out.Y.signals.values(:,1);
VXs=out.Xdot.signals.values(:,1);
VYs=out.Ydot.signals.values(:,1);
psis=out.psi.signals.values(:,1);
pwm_l=out.pwm_l.signals.values;
pwm_r=out.pwm_r.signals.values;
Vx_ref=out.Vx_ref.signals.values(:,1);
psi_ref=out.psi_ref.signals.values(:,1);
psidots=out.psidot.signals.values(:,1);
Vx=out.Vx.signals.values(:,1);

%% Simulation Results

robot_radius=0.3;                           % Robot dimension [m]

% Position
figure()
plot(Xs,Ys)
grid on
hold on
xlabel('X [m]')
ylabel('Y [m]')
title('Trajectory')
plot(X_goal,Y_goal,'o','MarkerSize',5,'MarkerEdgeColor', [0 0 0],'MarkerFaceColor',[1, 0.04, 0])
plot(ox,oy,'o','MarkerSize',5,'MarkerEdgeColor', [1 0 0],'MarkerFaceColor',[1, 0.55, 0])
plot(X_0,Y_0,'o','MarkerSize',5,'MarkerEdgeColor',[0 0.3 0.3],'MarkerFaceColor', [0.4, 0.7, 0.2])
plot(Xs(end),Ys(end),'o','MarkerSize',5,'MarkerEdgeColor', [0 0 1],'MarkerFaceColor',[0 0.44 1])
ang=0:0.01:2*pi;
xr=0.7*cos(ang);
yr=0.7*sin(ang);
for i=1:length(ox)
plot(xr+ox(i),yr+oy(i),'--','Color',[1, 0.55, 0])
end
axis equal
%% Results

figure()
plot(ts,Xs,'LineWidth',1.25,'Color',[0.7, 0 ,0])
grid on
title('X position')
xlabel('Time [s]')
ylabel('X [m]')


figure()
plot(ts,Ys,'LineWidth',1.25,'Color',[0, 0.7 ,0])
title('Y position')
xlabel('Time [s]')
ylabel('Y [m]')
grid on

%PWM
figure()
plot(ts,pwm_l,'LineWidth',1.25)
grid on
hold on
plot(ts,pwm_r,'LineWidth',1.25)
xlabel('Time [s]')
ylabel('PWM [\mus]')
sgtitle('PWM')
legend('PWM_l','PWM_r')

%Speed
figure()
plot(ts,VXs,'LineWidth',1.25)
grid on
hold on
plot(ts,VYs,'LineWidth',1.25)
xlabel('Time [s]')
ylabel('V [m/s]')
sgtitle('Speed')
legend('VX','VY')
ylim([-0.3 0.5])

figure()
plot(ts,psidots,'LineWidth',1.25,'Color',[1 0.5 0])
grid on
xlabel('Time [s]')
ylabel('\psidot [rad/s]')
sgtitle('Angular speed')
ylim([-2 2.5])

%Orientation
figure()
plot(ts,psis*180/pi,'LineWidth',1.25,'Color',[0 0.7 0])
grid on
xlabel('Time [s]')
ylabel('\psi [deg]')
title('Angular position')
ylim([-180 180])

%% REFERENCE
%Speed
figure()
plot(ts,Vx,'LineWidth',1.25,'Color',[0, 0.7 ,0])
grid on
hold on
plot(ts,Vx_ref,'LineWidth',1.25)
xlabel('Time [s]')
ylabel('V [m/s]')
sgtitle('Velocity')
legend('Vx', 'Reference')
ylim([-0.3 0.5])

%Orientation
figure()
plot(ts,psis*180/pi,'LineWidth',1.25,'Color',[1, 0.7 ,0])
grid on
hold on
plot(ts, psi_ref*180/pi,'LineWidth',1.25)
xlabel('Time [s]')
ylabel('\psi [deg]')
title('Angular position')
legend('\psi', 'Reference')
ylim([-90 90])

%% REFERENCES ONLY
figure()
subplot(2,1,1)
grid on
hold on
plot(ts,Vx_ref,'LineWidth',1.25)
xlabel('Time [s]')
ylabel('V [m/s]')
title('Velocity')
subplot(2,1,2)
grid on
hold on
plot(ts, psi_ref*180/pi,'LineWidth',1.25,'Color',[0 0.7 0])
xlabel('Time [s]')
ylabel('\psi [deg]')
title('Angular position')
sgtitle('Reference')