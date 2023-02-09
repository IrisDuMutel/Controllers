%%% Models based on the examples in:
%%% A simulative study on active disturbance rejection control (ADRC)
%%% as a control tool for practitioners


%% %% First order LADRC siso

C = [1 0];
A = [0 1; 0 0];
K = 1;
T = 1;
b0 = K/T;
Tsettle = 1;
Kp = 4/Tsettle;
B = [b0; 0];
Scl = -Kp;
S_eso = 10*Scl;
L = [-2*S_eso; S_eso^2];


%% %% Second order LADRC siso

C = [1 0 0];
A = [0 1 0; 0 0 1; 0 0 0];
K = 1;
T = 1;
D = 1;
a = T^2;
b = 2*D*T;
b0 = K/T^2;
Tsettle = 2;
B = [0; b0; 0];
Scl = -6/Tsettle;
S_eso = 10*Scl;
Kp = Scl^2;
Kd = -2*Scl;
L = [-3*S_eso; 3*S_eso^2; -S_eso^3];

