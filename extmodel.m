% % % % Going from continuous to discrete with delta_t
Ac = [0 1 0; 3 0 1; 0 1 0];
Bc = [1; 1; 3];
Cc = [0 1 0 ];
Dc = zeros(1,1);
Delta_t = 1;
[Ad,Bd,Cd,Dd] = c2dm(Ac,Bc,Cc,Dc,Delta_t);

% [m1,n1] = size(Cd);
% [n1,n_in] = size(Bd);
% A_e = eye(n1+m1,n1+m1);
% A_e(1:n1,1:n1) = Ad;
% A_e(n1+1:n1+m1,1:n1) = Cd*Ad;
% B_e = zeros(n1+m1,n_in);
% B_e(1:n1) = Bd;
% B_e(n1+1:n1+m1,:) = Cd*Bd;
% C_e = zeros(m1,n1+m1);
% C_e(:,n1+1:n1+m1) = eye(m1,m1);

Ap = 0.8;Bp = 0.1;Cp = 1; Dd = 0;
Nc = 4; Np =10;
[Phi_Phi,Phi_F,Phi_R,BarRs,F,Phi,A_e,B_e,C_e] = mpcgains(Ap,Bp,Cp,Nc,Np)

figure()

x_next = [0.1; 0.2];
r_w = 0; u_next = 0;
x_m = C_e*x_next;
plot(1,x_next(1),'m-o')
hold on
plot(1,x_next(2),'b-o')
hold on
plot(1,u_next,'go');

% % % Basic propagation process
for i=2:5
    optimal_DeltaU = inv(Phi_Phi+r_w*eye(Nc))*(Phi_R-Phi_F*x_next)
    u_next = u_next + optimal_DeltaU(1);
    x_m = C_e*x_next;
    xm_next = Ap*x_m+Bp*u_next;
    y_next = xm_next;
    Delta_xm = xm_next-x_m;
    x_next = [Delta_xm; xm_next];
    plot(i,Delta_xm,'m-o')
    hold on
    plot(i,y_next,'b-o')
    hold on
    plot(i,u_next(1),'go');
end

%% Example 5

r_w = 10;
Phi'*F(:,2)
BarR = r_w*eye(Nc)

Ky = inv(Phi_Phi+BarR)*(Phi'*F(:,2));
Ky = Ky(1)
Kmpc = inv(Phi_Phi+BarR)*(Phi'*F);
Kmpc = Kmpc(1,:)
eig(A_e-B_e*Kmpc)

%% Example 6 
omega = 10;
Delta_t = 0.01;
Nc = 3;
Np = 20;
BarR = 0.5*eye(Nc);
num = omega^2;
den = [1 0.1*omega omega^2];
[Ac,Bc,Cc,Dc] = tf2ss(num,den);
[Ad,Bd,Cd,Dd] = c2dm(Ac,Bc,Cc,Dc,Delta_t);
[Phi_Phi,Phi_F,Phi_R,BarRs,F,Phi,A_e,B_e,C_e] = mpcgains(Ad,Bd,Cd,Nc,Np)

% % % Initial conditions
k = 10;
x_init = [0.1 0.2 0.3]';
% % % Basic propagation process

optimal_DeltaU = inv(Phi_Phi+BarR)*(Phi_R-Phi_F*x_init)
Ky = inv(Phi_Phi+BarR)*(Phi'*F(:,2));
Ky = Ky(1)
Kmpc = inv(Phi_Phi+BarR)*(Phi'*F);
Kmpc = Kmpc(1,:)
eig(A_e-B_e*Kmpc)


%% Example 7


