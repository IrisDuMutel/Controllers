mass = 1;
b = 3;
k = 1;


% LQR section
Q  = [1 0; 0 1]*1;
Qf = [1 0; 0 1];
R  = eye(1)*1e-2;

Ac = [0 1;...
     -k/mass -b/mass];
Bc = [0;1];
Cc = [1 0;0 1];
Dc = [0;0];

%%%% Discretizep
Delta_t = 0.01;
[Ad,Bd,Cd,Dd] = c2dm(Ac,Bc,Cc,Dc,Delta_t)

% MPC gains
Np = 4;
Nc = 4;
[Phi_Phi,Phi_F,Phi_R,BarRs,F,Phi,A_e,B_e,C_e] = mpcgains(Ad,Bd,Cd,Nc,Np)
Su0 = sparse(nbVar*nbData, nbVarPos*(nbData-1));
	Sx0 = kron(ones(nbData,1), speye(nbVar));
	M = B;
	for n=2:nbData
		id1 = (n-1)*nbVar+1:nbData*nbVar;
		Sx0(id1,:) = Sx0(id1,:) * A;
		id1 = (n-1)*nbVar+1:n*nbVar; 
		id2 = 1:(n-1)*nbVarPos;
		Su0(id1,id2) = M;
		M = [A*M(:,1:nbVarPos), M]; 
	end

% Fooooooorza Iris!
