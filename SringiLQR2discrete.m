mass = 1;
b = 0.01;
k = 0;


Ac = [0 1;...
     -k/mass -b/mass];
Bc = [0;1];
Cc = [1 0;0 1];
Dc = [0;0];

% Ac = [1 0.01;0 1];
% Bc = [0.0001; 0.001];
%%%% Discretizep
Delta_t = 0.001;
[Ad,Bd,Cd,Dd] = c2dm(Ac,Bc,Cc,Dc,Delta_t,'tustin')

% MPC gains
Np = 5;
Nc = 4;

% [Phi_Phi,Phi_F,Phi_R,BarRs,F,Phi,A_e,B_e,C_e] = mpcgains(Ad,Bd,Cd,Nc,Np);
A_e = full(kron(Ad, speye(2))); %Discrete nD
B_e = full(kron(Bd, speye(2))); %Discrete nD
C_e = eye(4);
[a1,a2] = size(A_e);
[b1,b2] = size(B_e);
[c1,c2] = size(C_e);
D_e = zeros(c1,b2);
Su0 = zeros(a1*Np, Nc*b2);
Sx0 = kron(ones(Np,1), speye(a1));
M = B_e;
id2 = 1;

for kk = a1+1:a1:Np*a1
   Sx0(kk:kk+a1-1,:) = Sx0(kk-a1:kk-1,:)*A_e;
end
v = Sx0*B_e;
Su0(:,1:b2) = v;
for i=b2+1:b2:Nc*b2 
    Su0(:,i:i+b2-1) = [zeros((i-1)/2*a1,b2);v(1:Np*a1-(i-1)/2*a1,:)]; 
end
Su0 = [zeros(a1,Nc*b2);Su0(1:end-a1,:)];
Su = full(Su0)
Sx = full(Sx0)
size(Su)
% Fooooooorza Iris!


% %Build Su and Sx transfer matrices
% nbData = 200; %Number of datapoints
% nbD = 5; %Size of the time window for MPC computation
% nbPoints = 5; %Number of keypoints
% nbVarPos = 2; %Dimension of position data (here: x1,x2)
% nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
% nbVar = nbVarPos * nbDeriv; %Dimension of state vector
% dt = 1E-2; %Time step duration
% rfactor = 1E-8; %dt^nbDeriv;	%Control cost in LQR
% 
% Su = sparse(nbVar*nbD, nbVarPos*(nbD-1));
% Sx = kron(ones(nbD,1), speye(nbVar));
% M = B_e;
% for n=2:nbD
% 	id1 = (n-1)*nbVar+1:nbD*nbVar;
% 	Sx(id1,:) = Sx(id1,:) * A_e;
% 	id1 = (n-1)*nbVar+1:n*nbVar; 
% 	id2 = 1:(n-1)*nbVarPos;
% 	Su(id1,id2) = M;
% 	M = [A_e*M(:,1:nbVarPos), M]; 
% end
% Su = full(Su)
% Sx = full(Sx)
% size(Su)


% LQR section
Q0 = zeros(a1*Np);
nbVarPos = 2; %Dimension of position data (here: x1,x2)
nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
nbVar = nbVarPos * nbDeriv; %Dimension of state vector
nbData = 5;
nbPoints=3;
tl = linspace(1,nbData,nbPoints+1);


Q  = eye(a1*Np)*1;
R  = eye(Nc*2)*1e-8;
