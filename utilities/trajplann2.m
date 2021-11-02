function [nedPos,nedVel,psiRef] = trajplann2(wpt,head,noPoints)

% Escludere gli if se i waypoints consecutivi sono uguali!!! OK!
% Aggiungere profilo velocità anche per lo yaw!! forse non possibile...

% REMEMBER TO ADD MORE EQUAL WAYPOINTS TO FORCE THE TRAJECTORY FROM THERE!
n = size(wpt,1);
nedPos = [];
psiRef = [];
nedVel = [];
nedVelx = [];
nedVely = [];
nedVelz = [];
maxVel = 2;%3;
riseFall = 1;%0.2; in meters
for ii = 1:n-1
%% Creating points in the NED frame...
    nedPos(end+1:end+noPoints,:) = ...
        [ linspace(wpt(ii,1),wpt(ii+1,1),noPoints)',...
        linspace(wpt(ii,2),wpt(ii+1,2),noPoints)',...
        linspace(wpt(ii,3),wpt(ii+1,3),noPoints)' ];
    
%% Creating heading rotation...    
    psiRef(end+1:end+noPoints) = linspace(head(ii),head(ii+1),noPoints)';
    
%% Creating velocities for each position point...
% Using trapezoidal profile
    % trapezoidal velocity... in 20cm it accelerates from 0 to 3m/s
%     nedVel_old = [ nedVelx, nedVely, nedVelz ];
    % X dimension
    delta_x = wpt(ii+1,1) - wpt(ii,1);
    if abs(delta_x) < 0.001
        nedVelx(end+1:end+noPoints,1) = zeros(noPoints,1);        
    elseif abs(delta_x) < riseFall*2
        midPointx = abs((wpt(ii+1,1) - wpt(ii,1))/2);
        nedVelx(end+1:end+noPoints,1) = ...
            [ linspace(0,sign(delta_x)*midPointx*maxVel/riseFall,noPoints/2)'; ...
            linspace(sign(delta_x)*midPointx*maxVel/riseFall,0,noPoints/2)' ];
    else
        nedVel_temp = [];
        nedVel_temp(end+1:end+noPoints,1) = sign(delta_x)*maxVel*ones(noPoints,1);
        pointsDensity = noPoints/abs(wpt(ii,1) - wpt(ii+1,1));
        risePoints = floor(pointsDensity*riseFall);
        fallPoints = floor(pointsDensity*riseFall);
        upPoints = noPoints - risePoints - fallPoints;
        nedVel_temp(1:risePoints,1) = ...
            linspace(0,sign(delta_x)*maxVel,risePoints);
        nedVel_temp(upPoints+risePoints+1:end,1) = ...
            linspace(sign(delta_x)*maxVel,0,fallPoints)';
%         nedVel_temp(upPoints+risePoints+1:end,1) = ...
%             sign(delta_x)*logspace(log10(maxVel),log10(1e-6),fallPoints)';
        nedVelx = [ nedVelx; nedVel_temp ];
    end
    % Y dimension
    delta_y = wpt(ii+1,2) - wpt(ii,2);
    if abs(delta_y) < 0.001
        nedVely(end+1:end+noPoints,1) = zeros(noPoints,1);        
    elseif abs(delta_y) < riseFall*2
        midPointy = abs((wpt(ii+1,2) - wpt(ii,2))/2);
        nedVely(end+1:end+noPoints,1) = ...
            [ linspace(0,sign(delta_y)*midPointy*maxVel/riseFall,noPoints/2)'; ...
            linspace(sign(delta_y)*midPointy*maxVel/riseFall,0,noPoints/2)' ];
    else
        nedVel_temp = [];
        nedVel_temp(end+1:end+noPoints,1) = sign(delta_y)*maxVel*ones(noPoints,1);
        pointsDensity = noPoints/abs(wpt(ii,2) - wpt(ii+1,2));
        risePoints = floor(pointsDensity*riseFall);
        fallPoints = floor(pointsDensity*riseFall);
        upPoints = noPoints - risePoints - fallPoints;
        nedVel_temp(1:risePoints,1) = ...
            linspace(0,sign(delta_y)*maxVel,risePoints);
        nedVel_temp(upPoints+risePoints+1:end,1) = ...
            linspace(sign(delta_y)*maxVel,0,fallPoints)';
%         nedVel_temp(upPoints+risePoints+1:end,1) = ...
%             sign(delta_y)*logspace(log10(maxVel),log10(1e-6),fallPoints)';
        nedVely = [ nedVely; nedVel_temp ];
    end
    % Z dimension
    delta_z = wpt(ii+1,3) - wpt(ii,3);
    if abs(delta_z) < 0.001
        nedVelz(end+1:end+noPoints,1) = zeros(noPoints,1);       
    elseif abs(delta_z) < riseFall*2
        midPointz = abs((wpt(ii+1,3) - wpt(ii,3))/2);
        nedVelz(end+1:end+noPoints,1) = ...
            [ linspace(0,sign(delta_z)*midPointz*maxVel/riseFall,noPoints/2)'; ...
            linspace(sign(delta_z)*midPointz*maxVel/riseFall,0,noPoints/2)' ];
    else
        nedVel_temp = [];
        nedVel_temp(end+1:end+noPoints,1) = sign(delta_z)*maxVel*ones(noPoints,1);
        pointsDensity = noPoints/abs(wpt(ii,3) - wpt(ii+1,3));
        risePoints = floor(pointsDensity*riseFall);
        fallPoints = floor(pointsDensity*riseFall);
        upPoints = noPoints - risePoints - fallPoints;
        nedVel_temp(1:risePoints,1) = ...
            linspace(0,sign(delta_z)*maxVel,risePoints);
        nedVel_temp(upPoints+risePoints+1:end,1) = ...
            linspace(sign(delta_z)*maxVel,0,fallPoints)';
%         nedVel_temp(upPoints+risePoints+1:end,1) = ...
%             sign(delta_z)*logspace(log10(maxVel),log10(1e-6),fallPoints)';
        nedVelz = [ nedVelz; nedVel_temp ];
    end

end
nedVel = [ nedVelx, nedVely, nedVelz ];
end



