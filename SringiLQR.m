mass = 1;
b = 3;
k = 1;


% LQR section
Q  = eye(2);
Qf = eye(2);
R  = eye(1)*10^-5;

x0 = [0 0]';
