%Constants

m = 1.3;
c3 = C(3);
c4 = C(4);
c9 = C(9);
c7 = C(7);

%State Space representation
A_lqr = [0 0 0 0 0 0 1 0 0 0 0 0
    0 0 0 0 0 0 0 1 0 0 0 0
    0 0 0 0 0 0 0 0 1 0 0 0
    0 0 0 0 0 0 0 0 0 1 0 0
    0 0 0 0 0 0 0 0 0 0 1 0
    0 0 0 0 0 0 0 0 0 0 0 1
    0 0 0 0 -g 0 0 0 0 0 0 0
    0 0 0 g 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0 0];
B_lqr = [0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    1/m 0 0 0 0 0
    0 1/m 0 0 0 0
    0 0 1/m 0 0 0
    0 0 0 c3 0 c4
    0 0 0 0 c7 0
    0 0 0 c4 0 c9];

C_lqr = [1 0 0 0 0 0 0 0 0 0 0 0
     0 1 0 0 0 0 0 0 0 0 0 0
     0 0 1 0 0 0 0 0 0 0 0 0
     0 0 0 1 0 0 0 0 0 0 0 0
     0 0 0 0 1 0 0 0 0 0 0 0
     0 0 0 0 0 1 0 0 0 0 0 0
     0 0 0 0 0 0 1 0 0 0 0 0
     0 0 0 0 0 0 0 1 0 0 0 0
     0 0 0 0 0 0 0 0 1 0 0 0
     0 0 0 0 0 0 0 0 0 1 0 0
     0 0 0 0 0 0 0 0 0 0 1 0
     0 0 0 0 0 0 0 0 0 0 0 1];
 
D_lqr = zeros(12,6);
 
 %Controlability and Observability
 
 Ctrb=ctrb(A_lqr,B_lqr); %computes the controlability matrix
 Obsr=obsv(A_lqr,C_lqr); %computes the observability matrix
 R_C = rank(Ctrb);
 
 
 %Q and R matrices
x_max = 1;
y_max = 1;
h_max = 3;
u_max = 5;
v_max = 5;
w_max = 0.5;
phi_max   = deg2rad(7);  % Maximum roll angle
theta_max = deg2rad(7);  % Maximum pitch angle
psi_max   = deg2rad(120); % Maximum yaw angle
Fx_max = 2; % maximum value for the roll
Fy_max = 2; % maximum value for the pitch
Fz_max = 2; % maximum value for the yaw

Q = 1*eye(12);
% 
% Q = [1/(x_max)^2 0 0 0 0 0 0 0 0 0 0 0;
%     0 1/(y_max)^2 0 0 0 0 0 0 0 0 0 0;
%     0 0 1/(h_max)^2 0 0 0 0 0 0 0 0 0;
%     0 0 0 1/(phi_max)^2 0 0 0 0 0 0 0 0;
%     0 0 0 0 1/(theta_max)^2 0 0 0 0 0 0 0;
%     0 0 0 0 0 1/(psi_max)^2 0 0 0 0 0 0;
%     0 0 0 0 0 0 1/(u_max)^2 0 0 0 0 0;
%     0 0 0 0 0 0 0 1/(v_max)^2 0 0 0 0;
%     0 0 0 0 0 0 0 0 1/(w_max)^2 0 0 0;
%     0 0 0 0 0 0 0 0 0 1 0 0;
%     0 0 0 0 0 0 0 0 0 0 1 0;
%     0 0 0 0 0 0 0 0 0 0 0 1];

R =1*eye(6,6);

% R = [1/(Fx_max)^2 0 0 0 0 0;
%      0 1/(Fy_max)^2 0 0 0 0;
%      0 0 1/(Fz_max)^2 0 0 0;
%      0 0 0 1 0 0;
%      0 0 0 0 1 0;
%      0 0 0 0 0 1];


 
 %Optimal K
[K_lqr, S, e] = lqr(A_lqr, B_lqr, Q, R)

 AQ = A_lqr-B_lqr*K_lqr;
 eig(AQ)
