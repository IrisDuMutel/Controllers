mass = 1;
b = 3;
k = 1;




Ac = [0 1;...
     -k/mass -b/mass];
Bc = [0;1];
Cc = [1 0;0 1];
Dc = [0;0];

%%%% Discretizep
Delta_t = 0.01;
[Ad,Bd,Cd,Dd] = c2dm(Ac,Bc,Cc,Dc,Delta_t)

% MPC gains
Np = 5;
Nc = 5;
[Phi_Phi,Phi_F,Phi_R,BarRs,F,Phi,A_e,B_e,C_e] = mpcgains(Ad,Bd,Cd,Nc,Np)
[a1,a2] = size(A_e);
[b1,b2] = size(B_e);
[c1,c2] = size(C_e);
Su0 = sparse(a1*Np, Nc*b2);
Sx0 = kron(ones(Np,1), speye(a1));
M = B_e;
id2 = 1;
% Su0(b1+1:end,:) = Phi;
for kk = a1+1:a1:Np*a1
   Sx0(kk:kk+a1-1,:) = Sx0(kk-a1:kk-1,:)*A_e;
end
v = Sx0*B_e;
Su0(:,1:b2) = v;
for i=b2+1:b2:Nc*b2 
    Su0(:,i:i+b2-1) = [zeros(i-1,1);v(1:Np*a1-i+1,1)]; %Toeplitz matrix
end
Su0 = [zeros(a1,Nc*b2);Su0(1:end-a1,:)];
Su = full(Su0); Sx = full(Sx0);
% Fooooooorza Iris!

% LQR section
Q  = eye(a1*Np)*1;
Qf = [1 0; 0 1];
R  = eye(1)*1e-2;
