% r = 0.02;
% B = 0.185;
% dt = 0.01;
k = 0;
b = 0.01;
mass = 1;


toll = 3*10^-5;

A = [0 1;...
     -k/mass -b/mass];
B = [0;1];
% state = [x,x_dot];
% J = state*Qf*state'+state*Q*state'+input*R*input';
% cmd = zeros(1,1);
% x_e = [state_iLQR];
% if abs(Jold-J)/Jold > toll
x = zeros(4,1);
xr = [];
x_deg = ones(1,25);
reference = [zeros(1,25),x_deg,zeros(1,25),x_deg,zeros(1,25),x_deg,zeros(1,25),x_deg,zeros(1,25),x_deg,zeros(1,25),x_deg,zeros(1,25),x_deg,zeros(1,25),x_deg,];
for i=1:200
    xr(i,:) = x;
    Mu = ones(20,1)*reference(i);
    cmd_e = (Su'*Q*Su+R)\Su'*Q*(Mu-Sx*x);
    cmd = cmd_e(1:2);
    x = A_e*x + B_e*cmd;
    
end
plot(xr(:,4));
hold on 
grid on;

    