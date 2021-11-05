%Constants

m = 1.3;
c3 = C(3);
c4 = C(4);
c9 = C(9);
c7 = C(7);

%State Space representation
A_lqr = [0 1 0 0 0 0 0 0
    0 0 0 0 0 0 0 0
    0 0 0 0 0 1 0 0
    0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 1
    0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0];
B_lqr = [0 0 0 0 0 0
    0 0 1/m 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 c3 0 c4
    0 0 0 0 c7 0
    0 0 0 c4 0 c9];

C_lqr = [1 0 0 0 0 0 0 0
     0 1 0 0 0 0 0 0
     0 0 1 0 0 0 0 0
     0 0 0 1 0 0 0 0
     0 0 0 0 1 0 0 0
     0 0 0 0 0 1 0 0
     0 0 0 0 0 0 1 0
     0 0 0 0 0 0 0 1];
 
D_lqr = zeros(8,6);
 
 %Controlability and Observability
 
 Ctrb=ctrb(A_lqr,B_lqr); %computes the controlability matrix
%  Obsr=obs(A,C); %computes the observability matrix
 R_C = rank(Ctrb);
 
 
 %Q and R matrices
phi_max   = deg2rad(7);  % Maximum roll angle
theta_max = deg2rad(7);  % Maximum pitch angle
psi_max   = deg2rad(4); % Maximum yaw angle
h_max     = 1;
w_max     = 0.2;
Fx_max = 2; % maximum value for the roll
Fy_max = 2; % maximum value for the pitch
Fz_max = 2; % maximum value for the yaw

Q = 1*eye(8);

% Q = [1/(h_max)^2 0 0 0 0 0 0 0
%     0 0 1/(phi_max)^2 0 0 0 0 0
%     0 0 0 1/(theta_max)^2 0 0 0 0
%     0 0 0 0 1/(psi_max)^2 0 0 0];

% Q = [1/(h_max)^2 0 0 0 0 0 0 0;
%     0 1 0 0 0 0 0 0;
%     0 0 1/(phi_max)^2 0 0 0 0 0;
%     0 0 0 1/(theta_max)^2 0 0 0 0;
%     0 0 0 0 1/(psi_max)^2 0 0 0;
%     0 0 0 0 0 1 0 0;
%     0 0 0 0 0 0 1 0;
%     0 0 0 0 0 0 0 1];

R =140*eye(6,6);
% R = [1 1 1 1 1 1];
% R = [1/(Fx_max)^2 0 0 0 0 0;
%      0 1/(Fy_max)^2 0 0 0 0;
%      0 0 1/(Fz_max)^2 0 0 0;
%      0 0 0 1 0 0;
%      0 0 0 0 1 0;
%      0 0 0 0 0 1];
%  R = [1/(Fx_max)^2
%      1/(Fy_max)^2
%      1/(Fz_max)^2
%      1
%      1
%      1];

 
 %Optimal K
[K_lqr, S, e] = lqr(A_lqr, B_lqr, Q, R, 0)

 AQ = A_lqr-B_lqr*K_lqr;
 eig(AQ)
