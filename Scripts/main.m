syms vr theta T real
x0 = [0;0;0];
u0 = [0;0];
A = [1 0 -vr*sin(theta)*T; 0 1 vr*cos(theta)*T; 0 0 1];
B = [cos(theta)*T 0; sin(theta)*T 0; 0 T];
C = [1 1 1 ];
D = [0 0];
A0 = [1 0 u0(1)*sin(x0(3)); 0 1 u0(1)*cos(x0(3)); 0 0 1];
B0 = [cos(x0(3)) 0; sin(x0(3)) 0; 0 1];
Q = diag([1 1 0.5]);
R = eye(2)*0.1;
N = 4;
vmin = 0.4;
vmax = -0.4;
omegamin = -0.2;
omegamax = 0.2; 

theta = 0;
vr = 2;
T = 0.1;

Av = subs(A);
Bv = subs(B);
Cv = eye(3);

rank(ctrb(Av,Bv));
% When not in motion, the system is incontrollable

eig(Av);

M = [A0;A0^2;A0^3;A0^4];
CONV = [B0 zeros(size(B0)) zeros(size(B0)) zeros(size(B0));
        A0*B0 B0 zeros(size(B0)) zeros(size(B0));
        A0^2*B0 A0*B0 B0 zeros(size(B0));
        A0^3*B0 A0^2*B0 A0*B0 B0];
    
Q_hat = blkdiag(Q,Q,Q,Q);
R_hat = blkdiag(R,R,R,R);

H = CONV'*Q_hat*CONV + R_hat;
F = CONV'*Q_hat*M;

K = H\F;
K_mpc = K(1,:);

X_pred = M*x_actual + CONV*u_actual;

% STEP 1) obtain the model
% Xdot = Vx cos(theta)
% Ydot = Vx sin(theta)
% thetadot = omega

% u = [Vx omega]';
% x = [x y theta]';


%%