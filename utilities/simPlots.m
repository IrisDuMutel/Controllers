 %% Plots script
%% N-E plot
figure
% plot(realVel.time,realVel.data(:,2),'b');
grid on
hold on
plot(pos_ref(:,2),pos_ref(:,1),'r')
hold on
plot(out.realPos.Data(:,2),out.realPos.Data(:,1));
for ii = 1:size(wpt,1)
    plot(wpt(ii,2),wpt(ii,1),'o','MarkerFaceColor','y');
    % Naming the waypoints...
    if ii ~= 1 && ii ~= size(wpt,1)
        text(wpt(ii,2),wpt(ii,1), sprintf('wpt %d', ii))
    end
end
% for ii = 1:size(pos_ref,1) % Plotting only 1 value of tot...
%     if mod(ii,20) == 0
%         plot(pos_ref(ii,1),pos_ref(ii,2),'ro')
%     end
% end
legend('Reference','Actual trajectory')
xlabel('East [m]');
ylabel('North [m]');
set(gca,'FontSize',12)
axis('equal')

%% 3D plot NED
figure
hold on
plot3(out.realPos.Data(:,2),out.realPos.Data(:,1),out.realPos.Data(:,3))
for ii = 1:size(wpt,1)
    plot3(wpt(ii,2),wpt(ii,1),wpt(ii,3),'ro','MarkerFaceColor','r');
end
grid on
legend('Actual trajectory','Waypoints')
xlabel('East');
ylabel('North');
zlabel('Down');
% title('North-East-Down plot');

%% 3D plot xyz
% temp = [lgnd; lgnd.ItemText];
% set(temp, 'FontSize', 14)
% set(temp, 'FontName', 'Times New Roman')
figure
hold on
plot3(pos_ref(:,1),-pos_ref(:,2),-pos_ref(:,3))
plot3(out.realPos.Data(:,1),-out.realPos.Data(:,2),-out.realPos.Data(:,3))
for ii = 1:size(wpt,1)
    plot3(wpt(ii,1),-wpt(ii,2),-wpt(ii,3));
    end
for ii = 1:size(wpt,1)
    plot3(wpt(ii,1),-wpt(ii,2),-wpt(ii,3),'ro','MarkerFaceColor','r');
    if ii ~= size(wpt,1)
        text(wpt(ii,1),-wpt(ii,2),-wpt(ii,3), sprintf('wpt %d', ii))
    end
end
grid on
% for jj = 1:size(realPos.Data,1)
%     vectComp(jj,:) = [ sign(realPos.Data(jj,1)), sign(-realPos.Data(jj,2)), ...
%         sign(-realPos.Data(jj,3)) ];
% end
% quiver3(realPos.Data(:,1),-realPos.Data(:,2),-realPos.Data(:,3),...
%     vectComp(:,1),vectComp(:,2),vectComp(:,3));
legend('Actual trajectory', 'Reference trajectory')
xlabel('$x [m]$');
ylabel('$y [m]$');
zlabel('$z [m]$');
% title('xyz plot');
view([30,30]);

%% ref vs actual plot
figure
hold on
plot(out.realPos.time,out.realPos.data(:,1),'b')
plot(t_ref,pos_ref(:,1),'--r')
grid on
xlabel('Time [$s$]')
ylabel('North [$m$]')
legend('Actual position','Reference position')

figure
hold on
plot(out.realPos.time,out.realPos.data(:,2),'b')
plot(t_ref,pos_ref(:,2),'--r')
grid on
xlabel('Time [$s$]')
ylabel('East [$m$]')
legend('Actual position','Reference position')

figure
hold on
plot(out.realPos.time,out.realPos.data(:,3),'b')
plot(t_ref,pos_ref(:,3),'--r')
grid on
xlabel('Time [$s$]')
ylabel('Down [$m$]')
legend('Actual position','Reference position')

% Steady state error
start_t = find(t_ref - 3 == 0);
indx = find(t_ref(start_t(1)) - out.realPos.time <= 1e-2);
h_mean = mean(out.realPos.data(indx,3));
ss_err = pos_ref(end,3) - h_mean;

%% Command input plot
figure
plot(cmd.time,cmd.data(:,3))
grid on
xlabel('Time [$s$]');
ylabel('Thrust [$N$]');

figure
hold on
plot(cmd.time,cmd.data(:,4),'b');
plot(cmd.time,cmd.data(:,5),'r');
plot(cmd.time,cmd.data(:,6),'g');
grid on
xlabel('Time [$s$]')
ylabel('Moment [$Nm$]')
legend('$\tau_{x}$','$\tau_{y}$','$\tau_{z}$')


%% Euler angles behavior separated
%phi
figure
title('Response to a step input')
plot(eulAng.time,eulAng.data(:,1))
hold on
plot(eulAng.time,step(:,1))
grid on
xlabel('Time(s)')
ylabel('Angle [$rad$]')

%theta
figure
title('Response to a step input')
plot(eulAng.time,eulAng.data(:,2))
hold on
plot(eulAng.time,theta_step(:,1))
grid on
xlabel('Time(s)')
ylabel('Angle [$rad$]')

%psi
figure
title('Response to a step input')
plot(eulAng.time,psi_ref)
hold on
plot(eulAng.time,eulAng.data(:,3))
grid on
xlabel('Time(s)')
ylabel('Yaw angle [$rad$]')
legend('$\psi_{des}$','$\psi_{est}$')

figure
plot(eulAng.time,eulAng.data(:,1))
hold on
grid on
plot(eulAng.time,eulAng.data(:,2))
plot(eulAng.time,eulAng.data(:,3))
xlabel('Time [$s$]')
ylabel('Angle [$rad$]')
legend('$\phi$','$\theta$','$\psi$')

%psi
figure
title('Response to a step input')
plot(eulAng.time,psi_des)
hold on
plot(eulAng.time,eulAng.data(:,3))
grid on
xlabel('Time(s)')
ylabel('Yaw angle [$rad$]')
legend('$\psi_{des}$','$\psi_{est}$')
plot(t_ref,psi_dot)
plot(eulAng.time,psidot_sim(:,1))
%x
figure
title('Response to a step input')
plot(realPos.time,realPos.data(:,1))
hold on
plot(t_ref,pos_ref(:,1))
grid on
xlabel('Time(s)')
ylabel('Distance y [$m$]')

%% Euler angles subplot
figure()
hold on
h1=subplot(3,1,1);
hold on
plot(eulAng.time,eulAng.data(:,1))
grid on
h2=subplot(3,1,2); 
hold on
plot(eulAng.time,eulAng.data(:,2))
ylabel('Angle [rad]')
grid on
h3=subplot(3,1,3);
hold on
plot(eulAng.time,eulAng.data(:,3))
xlabel('Time [s]')
grid on

%% Euler angles subplot to compare with references
figure()
set(gca,'FontSize', 14);
hold on
h1=subplot(3,1,1);
set(gca,'FontSize',12)
hold on
plot(eulAng.time,phi_des(:,1),'LineWidth',4)
plot(eulAng.time,eulAng.data(:,1),'LineWidth',2)
legend('$\phi_{des}$','$\phi_{sim}$','FontSize',12)
grid on
h2=subplot(3,1,2); 
set(gca,'FontSize',12)
hold on
plot(eulAng.time,theta_des(:,1),'LineWidth',4)
plot(eulAng.time,eulAng.data(:,2),'LineWidth',2)
legend('$\theta_{des}$','$\theta_{sim}$','FontSize',12)
ylabel('Angle [rad]','FontSize',14)
grid on
h3=subplot(3,1,3);
set(gca,'FontSize',12)
hold on
plot(eulAng.time,psi_des(:,1),'LineWidth',4)
plot(eulAng.time,eulAng.data(:,3),'LineWidth',2)
legend('$\psi_{des}$','$\psi_{sim}$','FontSize',12)
xlabel('Time [s]','FontSize',14)
grid on
%% vel ref vs actual vel
figure
hold on
plot(realVel.time,realVel.data(:,1),'b')
plot(t_ref,vel_ref(:,1),'--r')
grid on
xlabel('Time [$s$]')
ylabel('Velocity (North) [$m/s$]')
legend('Actual NED velocity','Reference NED velocity')

figure
hold on
plot(realVel.time,realVel.data(:,2),'b')
plot(t_ref,vel_ref(:,2),'--r')
grid on
xlabel('Time [$s$]')
ylabel('Velocity (East) [$m/s$]')
legend('Actual NED velocity','Reference NED velocity')

figure
hold on
plot(realVel.time,realVel.data(:,3),'b')
plot(t_ref,vel_ref(:,3),'--r')
grid on
xlabel('Time [$s$]')
ylabel('Velocity (Down) [$m/s$]')
legend('Actual NED velocity','Reference NED velocity')

%% Velocities subplot
figure()
hold on
h1=subplot(3,1,1);
hold on
plot(realVel.time,realVel.data(:,1),'b')
plot(t_ref,vel_ref(:,1),'--r')
ylabel('Vel (North) [$m/s$]')
grid on
h2=subplot(3,1,2); 
hold on
plot(realVel.time,realVel.data(:,2),'b')
plot(t_ref,vel_ref(:,2),'--r')
ylabel('Vel (East) [$m/s$]')
grid on
h3=subplot(3,1,3);
hold on
plot(realVel.time,realVel.data(:,3),'b')
plot(t_ref,vel_ref(:,3),'--r')
ylabel('Vel (Down) [$m/s$]')
xlabel('Time [s]')
grid on
%% vel NE vs actual NE vel (one plot)
figure
hold on
plot(realVel.time,realVel.data(:,1),'b')
plot(t_ref,vel_ref(:,1),'--r')
grid on

plot(realVel.time,realVel.data(:,2),'color','c')
plot(t_ref,vel_ref(:,2),'--g')
xlabel('Time [$s$]')
ylabel('Velocity [$m/s$]')
legend('Simulated North velocity','Reference North velocity',...
    'Simulated East velocity','Reference East velocity')
%% psi and psi_dot ref vs actual values from simulation
figure
hold on
plot(t_ref,psi,'r')
plot(realVel.time,psi_sim(:,1),'color','b')
plot(t_ref,psi_dot,'m')
plot(realVel.time,psidot_sim(:,1),'color','g') 
set(gca,'YTick',-pi:pi/6:5*pi/2)
xlabel('Time [s]')
ylabel('Angle [rad]')
set(gca,'YTickLabel',{'$-\pi$','$-5\pi/6$','$-2*\pi/3$','$-\pi/2$','$-\pi/3$','$-\pi/6$','0','$\pi/6$','$\pi/3$','$\pi/2$','$2\pi/3$','$5\pi/6$','$\pi$','$7\pi/6$','$4\pi/3$','$3\pi/2$','$5\pi/3$','$11\pi/6$','$2\pi$','$13\pi/6$','$7\pi/3$','$5\pi/2$'})
legend('$\psi_{ref}$','$\psi_{Sim}$','$\dot{\psi}_{ref}$','$\dot{\psi}_{Sim}$')
grid on
%% ref pos vs actual plot
figure
hold on
plot(realPos.time,realPos.data(:,1),'b')
plot(t_ref,pos_ref(:,1),'--r')

plot(realPos.time,realPos.data(:,2),'c')
plot(t_ref,pos_ref(:,2),'--','color',[0.8500, 0.3250, 0.0980])

plot(realPos.time,realPos.data(:,3),'g')
plot(t_ref,pos_ref(:,3),'--m')
grid on
xlabel('Time [$s$]')
ylabel('Position [$m$]')
legend('Real North position','Reference North position','Real East position','Reference East position','Real Down position','Reference Down position')
%% Positions h
figure()
hold on
plot(t_ref,pos_ref(:,3),'--m')
plot(realPos.time,realPos.data(:,3),'g')
grid on
xlabel('Time [$s$]')
ylabel('Altitude [$m$]')