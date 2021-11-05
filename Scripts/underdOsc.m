%%%% Continuous System %%%%
clear all;clc;close all
Ac = [0 1;-4 0];
Bc = [1 ; 0];
Cc = [0 1];
Dc = 0;

%%%% Discrete System %%%%
Delta_t = 0.1;
[Ad,Bd,Cd,Dd] = c2dm(Ac,Bc,Cc,Dc,Delta_t)

%%%% Prediction parameters %%%%
Nc = 3;     % Control Horizon
Np = 10;    % Prediction horizon
BarR = 0*eye(Nc);

%%%% Input signal amplitude is limited to +-25 by saturation

%%%% Matrices in the predictive control system %%%%
[Phi_Phi,Phi_F,Phi_R,BarRs,F,Phi,A_e,B_e,C_e] = mpcgains(Ad,Bd,Cd,Nc,Np)
[n,n_in] = size(B_e);
Ky = inv(Phi_Phi+BarR)*(Phi'*F(:,2));
Ky = Ky(1)
Kmpc = inv(Phi_Phi+BarR)*(Phi'*F);
Kmpc = Kmpc(1,:)
fprintf("Closed-loop eigenvalues:\n");
eig(A_e-B_e*Kmpc) % Closed loop eigenvalues

%%%% Simulation %%%%
%%%% a) Without saturation case:N_sim = 6;
N_sim = 16;
xm = [0.0008;
    1.0001];
Xf = zeros(n,1);
r = ones(N_sim,1);
u = 0;
y = 0;
for kk=1:N_sim
    DeltaU = inv(Phi_Phi+BarR)*(Phi_R*r(kk)-Phi_F*Xf);
    deltau = DeltaU(1,1);
    u = u+deltau;
    u1(kk) = u;
    y1(kk) = y;
    xm_old = xm;
    xm = Ad*xm+Bd*u;
    y = Cd*xm;
    Xf = [xm-xm_old; y];
end

%%%% Plotting %%%%
k = 0:(N_sim-1);
figure
% title('NO Saturation case')
subplot(211)
plot(k,y1)
xlabel('Sampling Instant')
legend('Output')
subplot(212)
plot(k,u1)
xlabel('Sampling Instant')
legend('Control')
xm = xm
%%%% b) With saturation case:
N_sim = 16;
% xm = [0;0];
xm = [0.0008;
    1.0001];
Xf = zeros(n,1);
r = ones(N_sim,1);
u = 0;
y = 0;
for kk=1:N_sim
    DeltaU = inv(Phi_Phi+BarR)*(Phi_R*r(kk)-Phi_F*Xf);
    deltau = DeltaU(1,1);
    u = u+deltau;
    if u >25
        u = 25;
    elseif u<-25
        u = -25;
    else
        u = u;
    end
    u1(kk) = u;
    y1(kk) = y;
    xm_old = xm;
    xm = Ad*xm+Bd*u;
    y = Cd*xm;
    Xf = [xm-xm_old; y];
end

%%%% Plotting %%%%
k = 0:(N_sim-1);
figure
% title('Saturation case')
subplot(211)
plot(k,y1)
xlabel('Sampling Instant')
legend('Output')
subplot(212)
plot(k,u1)
xlabel('Sampling Instant')
legend('Control')

%%%% c) With saturation and activated constraint case:
N_sim = 16;
% xm = [0;0];
xm = [0.0008;
    1.0001];
Xf = zeros(n,1);
xf_old = zeros(n,1);
r = ones(N_sim,1);
u_old = 0;
u = 0;
y = 0;
for kk=1:N_sim
    DeltaU = inv(Phi_Phi+BarR)*(Phi_R*r(kk)-Phi_F*Xf);
    deltau = DeltaU(1,1);
    u = u_old+deltau;
    if u >25
        u = 25;
        deltaU = 25-u_old;
    elseif u<-25
        u = -25;
        deltaU = -25-u_old;
    else
        u = u;
    end
    u1(kk) = u;
    y1(kk) = y;
    u_old = u;
    xm_old = xm;
    xm = Ad*xm+Bd*deltaU;
    y = Cd*xm;
    Xf = A_e*xf_old+B_e*deltaU;
    xf_old = Xf;
end

%%%% Plotting %%%%
k = 0:(N_sim-1);
figure
% title('Saturation case')
subplot(211)
plot(k,y1)
xlabel('Sampling Instant')
legend('Output')
subplot(212)
plot(k,u1)
xlabel('Sampling Instant')
legend('Control')

