%% Trajectory planner initializer
%  close all;
% LQR_complete;
%% Dubins Curves
%{
% TO USE THIS PART OF THE CODE, ONE MUST CREATE THE FOLLOWING INPUTS:
% Input:    wpt     : list of waypoints of the pattern
%           ToA     : list of times of arrival at the different waypoint of
%                     the trajectory.
%           theta   : Curve's heading definitiopn at every waypoint in wpt.
%           psi_des : Desired yaw reference
% 
% wpt = [ 0,0,0; 0,0,0; 0,0,0];
% ToA = [ 0, 5, 10];
%  
% theta = [0;0;0];
% psi_des = [];
% % % % Straight horizontal line
wpt = [ 0,0,0; 0,0,-2.5; 15,0,-2.5];
ToA = [ 0, 5, 15];
 
theta = [0;0;0];
psi_des = [];
% psi_des = [0;pi/4;pi/4];
% Square
% wpt = [ 0,0,0; 0,0,-2.5; 6,0,-2.5; 6,5,-2.5; 0,5,-2.5; 0,0,-2.5 ];
% ToA = [ 0, 5, 10, 15, 20, 25];
% theta = [0;0;pi/2;pi;3*pi/2;0;0];        
% psi_des = [];

% % Hexagon
% wpt = [ 0,0,0; 0,0,-2.5; 2,2,-2.5; 2,2*sqrt(8),-2.5; 0,2.6*sqrt(8),-2.5; -2,2*sqrt(8),-2.5;  -2,2,-2.5; 0,0,-2.5 ];
% ToA = [ 0, 5, 10, 15, 20, 25, 30, 35];
% theta = [0;0;pi/4;pi/2;3*pi/4;5*pi/4;3*pi/2;7*pi/4];        
% psi_des = [];

% Square (SHARP ANGLES) 
% wpt = [ 0,0,0; 0,0,-2.5; 0,3,-2.5; 3,0,-2.5; 0,-3,-2.5; -3,0,-2.5; 0,3,-2.5 ];
% ToA = [ 0, 5, 10, 15, 20, 25, 30  ];
% theta = [0;90*pi;90*pi/180;315*pi/180;225*pi/180;135*pi/180;pi/4];         
% psi_des = [];


% Snake
% wpt = [ 0,0,0; 0,0,-2.5; 3,-3,-2.5; 3,0,-2.5; -3,0,-2.5; -3,3,-2.5; 3,3,-2.5; ...
%     3,6,-2.5; -3,6,-2.5; -3,-3,-2.5; 3,-3,-2.5];
% theta = [0;2*pi;315*pi/180;pi/2;pi;pi/2;0;pi/2;pi;3*pi/2;0];
% ToA = [ 0, 5, 10, 15, 25, 30, 40, 45, 55, 70, 80 ];
% psi_des = [];

% % % S pattern
% wpt = [ 0,0,0; 0,0,-2.5; 12,0,-2.5; 24,0,-2.5];
% ToA = [ 0, 5, 25,45];
% theta = [0;pi/4;-pi/4;pi/4];
% psi_des = [];

% % % % Smooth zig zag
% wpt = [ 0,0,0; 0,0,-2.5; 8,0,-2.5; 13,5,-2.5; 20,5,-2.5];
% ToA = [ 0, 5, 20, 35, 50];
% theta = [0;0;0;pi/4;0];
% % psi_des = [0;0;2;0;0];
% psi_des = [];
% 
% % % M pattern
% wpt = [ 0,0,0;0,0,-2.5; 2,0,-2.5; 4,2,-2.5; 6,0,-2.5; 8,2,-2.5; 10,0,-2.5; 12,0,-2.5];
% ToA = [ 0, 5, 10,15,20,25,30,35];
% theta = [0; 0; pi/4;-pi/4;pi/4;-pi/4;0;0];
% psi_des = [];

% % Rise and turn at the same time
% wpt =     [ 0,0,0; 0,0,-0.5; 0,0,-1; 0,0,-1.5; 0,0,-2; 0,0,-2.5; 0,0,-3; ...
%     0,0,-2.5; 0,0,-2; 0,0,-1.5; 0,0,-1; 0, 0 -0.5; 0,0,0]; %; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5]
% ToA = [ 0,2,3, 4,5, 6, 7, 8, 9, 10, 11, 12 ,13];
% theta = 1*[0; pi/6; pi/3; pi/2; 2*pi/3; 5*pi/6; pi; 5*pi/6; 2*pi/3 ;pi/2;pi/3;pi/6;0];
% psi_des = [];

% %%% Infinity pattern
%  wpt = [ 0,0,0; 0,0,-2.5;2,0,-2.5;4,0,-2.5;4,0,-2.5;2,0,-2.5;0,0,-2.5];
%  ToA = [0;5;8;12;13;17;22];
%  theta = [0;-pi/4;pi/4;-pi/4;-3*pi/4;-5*pi/4;-3*pi/4];
% psi_des = [zeros(length(wpt),1)];
%%% Infinity pattern
 wpt = [ 0,0,0; 0,0,-2.5;1,-1,-2.5;2,0,-2.5;3,1,-2.5;4,0,-2.5;3,-1,-2.5;2,0,-2.5;1,1,-2.5;0,0,-2.5];
%  ToA = [0;2;5;10;15;20;25;30;35;40];
 %  ToA = [0;5;15;25;35;45];
ToA = [0;5;10;15;20;25;30;35;40;45];
 theta = [0;-pi/4;pi/4;pi/4;-pi/4;-3*pi/4;3*pi/4;3*pi/4;-3*pi/4;-pi/4];
psi_des = [zeros(length(wpt),1)];
% % % %Combined pattern
%  wpt = [ 0,0,0; 0,0,-1; 1,2,-1; 4,2,-1; 4,2,-2.5; 6,-2,-2.5; 9,-2,-2.5; 9,-2,-1;11,0,-1;11,0,0];
%  ToA = [0 ; 5; 10; 15; 20; 25;30;35;40;45];
% theta = [0; 0;pi/6;0;0;-pi/6;0;0;pi/6;0];
% psi_des = [];

[ pos_ref, vel_ref, t_ref, psi, psi_dot] = trajplann_dubins(wpt,ToA,theta,psi_des);    % Dubins Curves


% Plotting

figure()                                                            % produced positions in time
plot(t_ref,pos_ref(:,1), t_ref,pos_ref(:,2), t_ref,pos_ref(:,3));
grid on
legend('x','y','z')
figure()                                                            % produced velocities in time
plot(t_ref,vel_ref(:,1), t_ref,vel_ref(:,2), t_ref,vel_ref(:,3));
grid on
legend('$u_{NED}$','$v_{NED}$','$w_{NED}$');  
figure()                                                            % comparison between original trajectory and generated one
plot(pos_ref(:,1),pos_ref(:,2))
hold on
plot(wpt(:,1),wpt(:,2),'o')
legend('Generated trajectory','Original trajectory')
grid on
xlabel('East [m]');ylabel('North [m]')
figure()                                                            % turning rate
plot(t_ref,psi)
hold on
plot(t_ref,psi_dot)
legend('$\psi$','$\dot{\psi}$')
grid on
%}
%% Bezier Curves{
% TO USE THIS PART OF THE CODE, ONE MUST CREATE THE FOLLOWING INPUTS:
% Input:    wpt     : list of waypoints of the pattern
%           ToA     : list of times of arrival at the different waypoint of
%                     the trajectory.
%           PC      : Matrix containing the control points (two control
%                     points per segment).
%           psi_des : Desired heading at every waypoint in wpt.
%
% % % A point
% wpt = [ 0,0,0; 0,0,0; 0,0,0];
% ToA = [ 0, 5, 20];
%  
% PC = [0 0;
%     0 0;
%     0 0;
%     0 0];                    
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];
% % % % A line
wpt = [ 0,0,0; 0,0,-2.5; 15,0,-2.5];
ToA = [ 0, 5, 15];
 
PC = [0 0;
    0 0;
    0 15;
    0 0];                    
CP = [PC(1,2:end)' PC(2,2:end)';
    PC(3,2:end)' PC(4,2:end)'];
psi_des = [];
% psi_des = [0;pi/4;pi/4];
% 
% wpt = [ 0,0,0; 0,1,0; 0,1,0];
% ToA = [ 0, 5, 25];
%  
% PC = [0 0;
%     0 1;
%     0 0;
%     1 1];                    
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [0;0;0];
% % % Straight horizontal line
% wpt = [ 0,0,0; 0,0,-2.5; 15,0,-2.5];
% ToA = [ 0, 5, 15];
%  
% PC = [0 0;
%     0 0;
%     0 15;
%     0 0];                    
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];
% psi_des = [0; deg2rad(45);deg2rad(45)];

% Straight vertical line
% wpt = [ 0,0,0; 0,0,-2.5; 0,40,-2.5];
% ToA = [ 0, 5, 40];
%  
% PC = [0 0;
%     0 0;
%     0 0;
%     0 40];                    
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [0; pi/2; pi/2];
% psi_des = [0 ; 0 ; 0];

% % % % S pattern
% wpt = [ 0,0,0; 0,0,-2.5; 12,0,-2.5; 24,0,-2.5];
% ToA = [ 0, 5, 25,45];
%  
% PC = [0 2 14;
%     0 2 -2;
%     0 10 22;
%     0 2 -2];                    
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

% % Square counterclockwise
% wpt = [ 0,0,0; 0,0,-2.5; 6,0,-2.5; 6,6,-2.5; 0,6,-2.5; 0,0,-2.5 ];
% ToA = [ 0, 5, 15, 25, 35, 45];
% 
% PC = [0 0.25 6.25 5.75 -0.25;
%     0 -0.25 0.25 6.25 4.75;
%     0 5.75 6.25 0.25 -0.25;
%     0 -0.25 4.75 6.25 0.25];  
% PC = [0 0.5 6.5 5.5 -0.5;
%     0 -0.5 0.5 6.5 5.5;
%     0 5.5 6.5 0.5 -0.5;
%     0 -0.5 5.5 6.5 0.5]; 
% PC = [0 2 8 4 -2;
%     0 -2 2 8 4;
%     0 4 8 2 -2;
%     0 -2 4 8 2]; % to create circular pattern
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];

% % Square clockwise
% wpt = [ 0,0,0; 0,0,-2.5; 0,5,-2.5; 6,5,-2.5; 6,0,-2.5; 0,0,-2.5 ];
% ToA = [ 0, 5, 10, 15, 20, 25];
% 
% PC = [0 -0.25 0.25 6.25 5.75;
%     0 0.25 5.25 4.75 -0.25;
%     0 -0.25 5.75 6.25 0.25;
%     0 4.75 5.25 0.25 -0.25];                     
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];

% % Trapezoid
% wpt = [0,0,0;0,0,-2.5;6,0,-2.5;11,5,-2.5;5,5,-2.5;0,0,-2.5];
% ToA = [0,4,10,15,20,25];
% PC  = [0 0.25 6.25 10.75 4.75;
%        0 -0.25 0.25 5.25 4.75;
%        0 5.75 11.25 5.25 -0.25;
%        0 -0.25 4.75 5.25 0.25];
% CP  = [PC(1,2:end)' PC(2,2:end)';
%        PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

% % Zig Zag
% wpt = [ 0,0,0; 0,0,-2.5; 5,0,-2.5; 5,5,-2.5; 9,5,-2.5; 9,9,-2.5;12,9,-2.5; 12,12,-2.5 ];
% ToA = [ 0, 5, 15, 25,35,45,55,65];
% 
% PC = [0 0.25 5.25 5.25 9.25 9.25 12.25;
%     0 -0.25 0.25 5.25 5.25 9.25 9.25;
%     0 4.75 4.75 8.75 8.75 11.25 12.25;
%     0 -0.25 4.75 4.75 8.75 8.75 11.25];                   
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

% % % % Smooth Zig Zag
% wpt = [ 0,0,0; 0,0,-2.5; 8,0,-2.5; 13,5,-2.5; 20,5,-2.5];
% ToA = [ 0, 5, 20, 35, 50];
% PC  = [0 0.25 8.25 13.25;
%         0 -0.25 0.25 5.25;
%         0 7.75 12.75 19.75;
%         0 -0.25 4.75 5.25];                   
% CP  = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

% % % Hexagon
% wpt = [ 0,0,0; 0,0,-2.5; 2,2,-2.5; 2,2*sqrt(8),-2.5; 0,2.5*sqrt(8),-2.5; -2,2*sqrt(8),-2.5;  -2,2,-2.5; 0,0,-2.5 ];
% ToA = [ 0, 5, 10, 15, 20, 25, 30, 35];

% % Octagon
% wpt = [ 0,0,0; 0,0,-2.5; 3,0,-2.5; 5,2,-2.5; 5,5,-2.5; 3,7,-2.5;  0,7,-2.5; -2,5,-2.5; -2,2,-2.5; 0,0,-2.5];
% ToA = [ 0, 5, 15, 25, 35, 45, 55, 65, 75, 85];
% PC  = [0 0.25 3.25 5.25 4.75 2.75 -0.25 -2.25 -1.75;
%        0 -0.25 0.25 2.25 5.25 7.25 6.75 4.75 1.75;
%        0 2.75 4.75 5.25 3.25 0.25 -1.75 -2.25 -0.25;
%        0 -0.25 1.75 4.75 6.75 7.25 5.25 2.25 0.25];
% CP = [PC(1,2:end)' PC(2,2:end)';
%     PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

% % Square (SHARP ANGLES) 
% wpt = [ 0,0,0; 0,0,-2.5; 0,3,-2.5; 3,0,-2.5; 0,-3,-2.5; -3,0,-2.5; 0,3,-2.5 ];
% ToA = [ 0, 5, 10, 15, 20, 25, 30  ];

% % Step
% wpt = [ 0,0,0;0,0,-3; 0,0,-3 ];
% ToA = [ 0, 3, 5 ];

% wpt = [ 0,0,0; 0,0,0 ];
% ToA = [0, 30];

% % Butterfly
% wpt = [ 0,0,0; 0,0,-2.5; 3,-3,-2.5; 3,3,-2.5; -3,-3,-2.5; -3,3,-2.5; 0,0,-2.5 ];
% ToA = [ 0, 5, 10, 15, 20, 25, 30 ];
% PC = [0 0.25 3.25 2.75 -3.25 -2.75;
%     0 -0.25 -2.75 3.25 -2.75 3.25;
%     0 2.75 3.25 -2.75 -3.25 -0.25;
%     0 -3.25 2.75 -3.25 2.5 0.5];
% PC = [0 1 4 2 -4 -2;
%     0 -1 -2 4 -2 4;
%     0 2 4 -2 -4 -1;
%     0 -4 2 -4 2 1];
%  CP = [PC(1,2:end)' PC(2,2:end)';
%         PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

% % Snake
% wpt = [ 0,0,0; 0,0,-2.5; 3,-3,-2.5; 3,0,-2.5; -3,0,-2.5; -3,3,-2.5; 3,3,-2.5; ...
%     3,6,-2.5; -3,6,-2.5; -3,-3,-2.5; 3,-3,-2.5];
% ToA = [ 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30 ];
% ToA = [ 0, 5, 15, 25, 35, 45, 55, 65, 75, 85, 95 ];
% ToA = [ 0, 5, 10, 15, 25, 30, 40, 45, 55, 70, 80 ];
% PC = [0 0.25 3.25 2.75 -3.25 -2.75 3.25 2.75 -3.25 -2.75;
%       0 -0.25 -2.75 0.25 0.25 3.25 3.25 6.25 5.75 -3.25;
%       0 2.75 3.25 -2.75 -3.25 2.75 3.25 -2.75 -3.25 2.75;
%       0 -3.25 -0.25 -0.25 2.75 2.75 5.75 6.25 -2.75 -3.25];                 
%  CP = [PC(1,2:end)' PC(2,2:end)';
%         PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

% % M shape
wpt = [ 0,0,0;0,0,-2.5; 2,0,-2.5; 4,2,-2.5; 6,0,-2.5; 8,2,-2.5; 10,0,-2.5; 12,0,-2.5];
ToA = [ 0, 5, 10,15,20,25,30,35];
PC = [0 0.25 2.25 4.25 6.25 8.25 10.25;
        0 -0.25 0.25 2.25 -0.25 2.25 -0.25;
        0 1.75 3.75 5.75 7.75 9.75 11.75;
        0 -0.25 1.75 0.25 1.75 0.25 -0.25];                 
 CP = [PC(1,2:end)' PC(2,2:end)';
        PC(3,2:end)' PC(4,2:end)'];   
psi_des = [];

% % J shape
% wpt = [ 0,0,0;0,0,-2.5; 3,0,-2.5; 6,3,-2.5; 6,6,-2.5];
% ToA = [ 0, 5, 10,15,20];
% PC = [0 0.5 3.5 6.5;
%         0 -0.5 0.5 3.5;
%         0 2.5 5.5 6.5;
%         0 -0.5 2.5 5.5];                 
%  CP = [PC(1,2:end)' PC(2,2:end)';
%         PC(3,2:end)' PC(4,2:end)'];

% psi_des = [];
% % % Rise, hover then turn.
% wpt = [ 0,0,0; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; ...
%     0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5];%(17 components); 0,0,-2.5 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5]; %; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5]
% ToA = [ 0,2,3, 4,5, 6, 7, 8, 9, 10, 11, 12, 13, 14 ,15,16,17];% (17 components), 18,19,20,21,22,23,24,25,26,27,28,29 ];
% PC = [];
% % % Turn more than 2pi (17 components) 
% psi_des = 1*[0; 0; pi/6; pi/3; pi/2; 2*pi/3; 5*pi/6; pi; 7*pi/6; 4*pi/3 ;3*pi/2;5*pi/3;11*pi/6;2*pi;pi/6+2*pi;pi/3+2*pi;pi/2+2*pi];
% % % 
% % % To pi/2 and back (17 components)
% % psi_des = 1*[0;0;0;0;0;0;0; 0; pi/6; pi/3;pi/2; pi/3; pi/6; 0;0;0;0];
% % 
% % % To 2pi/3 and back
% psi_des = 1*[0; 0; pi/6; pi/3; pi/2;2*pi/3;pi/2; pi/3; pi/6; 0; 0 ;0;0;0;0;0;0]; %(17 components)

% % % Rise and turn at the same time
% wpt = [ 0,0,0; 0,0,-0.5; 0,0,-1; 0,0,-1.5; 0,0,-2; 0,0,-2.5; 0,0,-3; ...
%     0,0,-2.5; 0,0,-2; 0,0,-1.5; 0,0,-1; 0, 0 -0.5; 0,0,0]; %; 0,0,-2.5; 0,0,-2.5; 0,0,-2.5]
% ToA = [ 0,2,3, 4,5, 6, 7, 8, 9, 10, 11, 12 ,13];
% PC  = []; 
% psi_des = 1*[0; pi/6; pi/3; pi/2; 2*pi/3; 5*pi/6; pi; 5*pi/6; 2*pi/3 ;pi/2;pi/3;pi/6;0];
% % 
% %%% Infinity pattern
 wpt = [ 0,0,0; 0,0,-2.5;2,0,-2.5;4,0,-2.5;2,0,-2.5;0,0,-2.5];
 ToA = [0;5;15;25;35;45];
 PC  = [0 0 3 4 1;
        0 0.5 -1 0.5 -1;
        0 1 4 3 0;
        0 1 -0.5 1 -0.5];
     CP = [PC(1,2:end)' PC(2,2:end)';
        PC(3,2:end)' PC(4,2:end)'];
psi_des = [zeros(length(wpt),1)];

%%% combined pattern
%  wpt = [ 0,0,0; 0,0,-1; 1,2,-1; 4,2,-1; 4,2,-2.5; 6,-2,-2.5; 9,-2,-2.5; 9,-2,-1;11,0,-1;11,0,0];
%  ToA = [0 ; 5; 10; 15; 20; 25;30;35;40;45];
%  PC  = [0 0 1.25 4 4.25 6.25 9 9.25 11;
%         0 0 2.5 2 1.5 -2.5 -2 -1.5 0;
%         0 0.75 3.75 4 5.75 8.75 9 11 11;
%         0 1.5 2.5 2 -1.5 -2.5 -2 0 0];
%     CP = [PC(1,2:end)' PC(2,2:end)';
%         PC(3,2:end)' PC(4,2:end)'];
% psi_des = [];

[pos_ref, vel_ref, t_ref,psi,psi_dot] = trajplann_bezier(wpt,ToA,PC, psi_des);             % function corresponding to the usual patterns (first rise and then move in x-y)

%%%% plotting %%%%

figure
plot(t_ref,pos_ref(:,1), t_ref,pos_ref(:,2), t_ref,pos_ref(:,3));
grid on
legend('x','y','z')
figure
plot(t_ref,vel_ref(:,1), t_ref,vel_ref(:,2), t_ref,vel_ref(:,3));
grid on
legend('$u_{NED}$','$v_{NED}$','$w_{NED}$');  

figure()
plot(wpt(:,1),wpt(:,2),'r')
hold on
plot(pos_ref(:,1),pos_ref(:,2),'b')
hold on
for ii=1:length(CP)
plot(CP(ii,1),CP(ii,2),'*')
hold on
end
grid on
legend('Original pattern','Generated pattern','Control points')
xlabel('East [m]');ylabel('North [m]')

figure()
plot(t_ref,psi,t_ref,psi_dot)
legend('$\psi$','$\dot{\psi}$')
set(gca,'YTick',-pi:pi/6:2*pi) 
set(gca,'YTickLabel',{'-pi','-5pi/6','-2*pi/3','-pi/2','-pi/3','-pi/6','0','pi/6','pi/3','pi/2','2pi/3','5pi/6','pi','7pi/6','4pi/3','3pi/2','5pi/3','11pi/6','2pi'})
grid on
%}

