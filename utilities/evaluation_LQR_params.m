% X
kk = find(abs(realPos.data(:,1)-0.9)<0.0001)
rise_time = realPos.time(kk)-1

kk = find(abs(realPos.data(:,1)-0.95)<0.0001)
sett_time = realPos.time(kk)-1

kk = find(abs(realPos.data(:,1)>1))

% y
kk = find(abs(realPos.data(:,2)-0.9)<0.0001)
rise_time = realPos.time(kk)-1

kk = find(abs(realPos.data(:,2)-0.95)<0.0001)
sett_time = realPos.time(kk)-1

kk = find(abs(realPos.data(:,2)>1))


% psi
kk = find(abs(eulAng.data(:,3)-0.9)<0.0001)
rise_time = realPos.time(kk)-1

kk = find(abs(eulAng.data(:,3)-0.95)<0.0001)
sett_time = realPos.time(kk)-1

kk = find(eulAng.data(:,3)>1)

% z
kk = find(abs(realPos.data(:,3)-0.9)<0.001)
rise_time = realPos.time(kk)-1

kk = find(abs(realPos.data(:,3)-0.95)<0.001)
sett_time = realPos.time(kk)-1

maximo =max(realPos.data(:,3))
kk = find(realPos.data(:,3)==maximo)
overshoot = (realPos.data(kk,3)-1)*100