function [Phi_Phi,Phi_F,Phi_R,BarRs,F,Phi,A_e,B_e,C_e] = mpcgains(Ap,Bp,Cp,Nc,Np)

[m1,n1] = size(Cp);
[n1,n_in] = size(Bp);
A_e = eye(n1+m1,n1+m1);
A_e(1:n1,1:n1) = Ap;
A_e(n1+1:n1+m1,1:n1) = Cp*Ap;
B_e = zeros(n1+m1,n_in);
B_e(1:n1,:) = Bp;
B_e(n1+1:n1+m1,:) = Cp*Bp;
C_e = zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1) = eye(m1,m1);

n = n1+m1;
h(1:m1,:) = C_e;
F(1:m1,:) = C_e*A_e;
for kk = m1+1:m1:Np*m1
    h(kk:kk+m1-1,:) = h(kk-m1:kk-1,:)*A_e;
    F(kk:kk+m1-1,:) = F(kk-m1:kk-1,:)*A_e;
end

v = h*B_e;
Phi = zeros(Np*m1,Nc*n_in);
Phi(:,1:n_in) = v;
for i=n_in+1:n_in:Nc*n_in 
    Phi(:,i:i+n_in-1) = [zeros(i-1,1);v(1:Np*m1-i+1,1)]; %Toeplitz matrix
end

BarRs = ones(Np*m1,1);
Phi_Phi = Phi'*Phi;
Phi_F = Phi'*F;
Phi_R = Phi'*BarRs;


end

