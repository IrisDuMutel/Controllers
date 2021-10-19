mass = 1;
b = 3;
k = 1;


% LQR section
Q  = [1 0; 0 1];
Qf = [1 0; 0 1];
R  = eye(1)*5;

x0 = [0 0]';
