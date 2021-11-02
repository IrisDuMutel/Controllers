function [pos_ref,vel_ref,t_ref,psi,psi_dot] = trajplann_dubins(wpt,ToA,theta,psi_des)
%--------------------------------------------------------------------------
% Author: Iris David Du Mutel
% Year: 2019
% -------------------------------------------------------------------------
% This trajectoy planner works as follows:
        % The path is computed using Dubins curves. To do so, the
        % waipoints and the corresponding headings at each wpt are required.
        
        % It is allowed to perform vertical turns, hovering and hovering and
        % turning.
        
        % The linear velocities i.e. u,v and w NED are computed using a trapezoidal profile,
        % It must be noted that positions and velocities on this code are 
        % computed in NED reference frame. 
        
        % The angular velocity (psi_dot) is computed derivating psi on
        % time. Psi, is given by the dubins_curve.m code and corresponds to
        % the heading of the path at each point. Otherwise, psi can be
        % computed through psi_des, and therefore derivated from it. If
        % pis_des is left empty, theta will be used as psi_des.
        
        % THETA MUST NOT BE REMOVED. IT IS USED TO GENERATE THE GEOMETRIC
        % TRAJECTORY. PIS_DES IS OPTIONAL.

% Input:    wpt     : list of waypoints of the pattern
%           ToA     : list of times of arrival at the different waypoint of
%                     the trajectory.
%           theta   : list of headings at each wpt.
%           psi_des : desired yaw reference, if any.
%
% Output:   pos_ref : Set of coordinates of the final trajectory. Three
%                     components corresponding to x,y and z.
%           vel_ref : set of velocities vel_refx, vel_refy and vel_refz.
%           t_ref   : vector of time. From 0 to ToA(end) divided in
%                     noPoints for each segment.
%           psi     : vector of the heading at each point of the
%                     trajectory.
%           psi_dot : vector of the derivative of the yaw. 
%
%--------------------------------------------------------------------------

a     = 9.81/2;
t_ref = [0];
t_end = 0;
vel_refx = [];vel_refy = [];vel_refz = [];
pos_refx = [];pos_refy = [];pos_refz = [];
pos_ref      = [];

vel_oldz = [];
pos_x    = [];pos_y    = [];
psi      = [];psi_temp = [];psi_dot  = [];psi_end = [0];
psi_dot_end  = 0;psi_dot_temp = [];
noPoints = 1000;
stepsize = 1/noPoints;
n = size(wpt,1);                            % number of waypoints
r = 0.2;                                    % curvature radius (meters)
quiet = true;                               % true=no plots from dubins_curve.m
total_n_steps = 0;


for ii = 2:n
    noPoints = 1000;                                                                                % default
    % In which direction are we moving?
    delta_x = wpt(ii,1) - wpt(ii-1,1);                                                              % displacement in x-axis
    delta_y = wpt(ii,2) - wpt(ii-1,2);                                                              % displacement in y-axis
    delta_z = wpt(ii,3) - wpt(ii-1,3);                                                              % displacement in z-axis
    
    %%%% MOVEMENT IN THE Z AXIS %%%%
    if delta_z ~= 0
        %%% CREATION OF THE VELOCITY PROFILE IN Z%%%
        % When moving vertically, no horizontal movement is considered %
        t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);                                    % create time vector with noPoints elements between t_end and ToA
        dt      = t_ref(2+noPoints*(ii-2)) - t_ref(1+noPoints*(ii-2));                                   % time step in vector t_ref
        dt      = t_ref(end)-t_ref(end-1);
        %%%% CREATION OF VELOCITY POFILE (TRAPEZOIDAL). %%%%
        % When moving vertically, no horizontal movement is considered %
        v_max   = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a));...
            (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a)) ]/2;            % computation of maximum values of velocity
        v_max   = min(v_max)*sign(delta_z);
        delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;                                                  % time in which velocity is constant v_max
        t_star  = abs(v_max)/a;                                                                          % time in which the velocity reaches v_max
        t_hat   = t_star + delta_t;                                                                      % time in which velocity decreases from v_max
        vel_ref_temp(1:noPoints,1) = v_max;                                                              % initialization of the velocity vector
        up                 = find(t_ref(end-noPoints+1:end) - t_end < t_star);                           % components corresponding to points before t_star
        down               = find(t_ref(end-noPoints+1:end) - t_end > t_hat);                            % components corresponding to points after t_hat
        vel_ref_temp(up)   = sign(delta_z)*a*(t_ref(end-noPoints+up) - t_end);                           % creation of velocity gradient to v_max
        vel_ref_temp(down) = v_max - sign(delta_z)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));       % creation of velocity gradient from v_max
 
%         %%% CREATION OF THE POSITION VECTOR IN Z THROUGH INTEGRATION %%%
%         if isempty(pos_refz)
%             pos_refz(1,1) = 0;                                                                        % if it's empty, first component is 0
%         else
%             pos_refz(end+1,1) = pos_refz(end);                                                        % else, copy last component
%         end
%         for jj = 2:noPoints                                                                         % for the segment being analyzed
%             pos_refz(end+1,1) = (pos_refz(end)) + ...
%                 (vel_ref_temp(jj) + vel_ref_temp(jj-1))/2*dt;                                       % compute new positions integrating all along the segment(trapezoidal method)
%         end
        %%%% CREATION OF POSITION FROM INTEGRATION OF THE VELOCITY %%%%
        
        if isempty(pos_ref)                                                                              % check the z position vector
            pos_refz(1) = 0;                                                                             % if it's empty, first component is 0
        else
            pos_refz(1) = pos_ref(end,3);                                                                % else, copy last component to provide continuity
        end
        
        for jj = 2:noPoints                                                                              % for the segment being analyzed
            pos_refz(jj,1) = (pos_refz(jj-1,1)) + (vel_ref_temp(jj) + vel_ref_temp(jj-1))/2*dt;          % compute new positions integrating all along the segment(trapezoidal method of integration)
        end
        if length(pos_refz)>jj
            pos_refz(jj+1:end)=[];
        end
        %%% GENERATION OF PSI AND PSI_DOT WHILE MOVING IN Z %%%
        %%% Is there any psi_des?
        if isempty(psi_des) == 1                                                                    % using theta as pis_des
            psi_1    = theta(ii-1,:);                                                                   % heading in wpt(ii-1)
            psi_2    = theta(ii,:);                                                                     % heading in wpt(ii)
            psi_temp = zeros(noPoints,1);                                                               % initialization of psi_temp
            if psi_1 == psi_2                                                                           % if the angles are identical, no turning is performed
                psi_temp     = psi_1*ones(noPoints,1);
                psi_dot_temp = zeros(noPoints,1);
            else
                for i = 1:length(psi_temp)
                    psi_temp(i) = (psi_2-psi_1)/noPoints*i+psi_1;                                       % psi reference is a straight line that connects initial and final headings
                end
                psi_dot_temp    = zeros(noPoints,1);                                                    % initialization of psi_dot
                psi_dot_temp(1) = psi_dot_end;                                                          % to ensure continuity of the turning rate
                for i = 1:(noPoints-1)
                    psi_dot_temp(i+1,1) = (psi_temp(i+1)-psi_temp(i))/dt;                               % derivative of the angle
                end
            end
        % psi_des is not empty, generate yaw from it.
        else
            psi_1    = psi_des(ii-1,1);                                                                 % heading desired in wpt(ii-1)
            psi_2    = psi_des(ii,1);                                                                   % heading desired in wpt(ii)
            psi_temp = zeros(noPoints,1);                                                               % initialization of psi_temp
            if psi_1 == psi_2                                                                           % if the angles are identical, no turning is performed
                psi_temp     = psi_1*ones(noPoints,1);
                psi_dot_temp = zeros(noPoints,1);
            else
                for i = 1:length(psi_temp)
                    psi_temp(i) = (psi_2-psi_1)/noPoints*i+psi_1;                                       % psi reference is a straight line that connects initial and final headings
                end
                psi_dot_temp    = zeros(noPoints,1);                                                    % initialization of psi_dot
                psi_dot_temp(1) = psi_dot_end;                                                          % to ensure continuity of the turning rate
                for i = 1:(noPoints-1)
                    psi_dot_temp(i+1,1) = (psi_temp(i+1)-psi_temp(i))/dt;                               % derivative of the angle
                end
            end
        end
        
        %%% SAVING VARIABLES %%%
        vel_refx = [vel_refx; zeros(noPoints,1)];
        vel_refy = [vel_refy; zeros(noPoints,1)];
        vel_refz = [vel_refz; vel_ref_temp];
        pos_x = wpt(ii-1,1)*ones(noPoints,1);                                                         % no variation of x position
        pos_y = wpt(ii-1,2)*ones(noPoints,1); 
%         pos_x(end+1:end+noPoints,1) = wpt(ii-1,1)*ones(noPoints,1);                                   % same position in x
%         pos_y(end+1:end+noPoints,1) = wpt(ii-1,2)*ones(noPoints,1);                                   % same position in y
        pos_refx = [pos_x];
        pos_refy = [pos_y];
        total_n_steps = total_n_steps + noPoints;
        psi_dot  = [psi_dot;psi_dot_temp];                                                           % when going up or down, no yaw
        psi      = [psi;psi_temp];
        
        %%%% MOVEMENT IN THE X-Y PLANE %%%%
    % It can be one of two possibilities: hovering (and turning) or a linear
    % displacement following a Dubins curve
    else                                                                                            % this means that the movement is happening in the x-y plane
        p1   = [wpt(ii-1,1:2),theta(ii-1)];                                                         % starting point (of the segment)
        p2   = [wpt(ii,1:2),theta(ii)];                                                             % ending point (of the segment)
        
        %%% HOVERING CASE %%%
        if p1(1)== p2(1) && p1(2) == p2(2)                                                          % not moving but hovering
            t_ref(end:end+noPoints) = linspace(t_end,ToA(ii),noPoints+1);
            path = [wpt(ii-1,1)*ones(noPoints,1) wpt(ii-1,2)*ones(noPoints,1)];                     % x and y are the same as in the previous wpt
            if isempty(psi_des) == 1                                                                    % empty psi_des, using htetha instead
                psi_1    = theta(ii-1,:);                                                               % heading in wpt(ii-1)
                psi_2    = theta(ii,:);                                                                 % heading in wpt(ii)
                psi_temp = zeros(noPoints,1);                                                           % initialization of psi_temp
                if psi_1 == psi_2                                                                       % if the angles are identical, no turning is performed
                    psi_temp     = psi_1*ones(noPoints,1);
                    psi_dot_temp = zeros(noPoints,1);
                else
                    for i = 1:length(psi_temp)
                        psi_temp(i) = (psi_2-psi_1)/noPoints*i+psi_1;                                   % psi reference is a straight line that connects initial and final headings
                    end
                    psi_dot_temp    = zeros(noPoints,1);                                                % initialization of psi_dot
                    psi_dot_temp(1) = psi_dot_end;                                                      % to ensure continuity of the turning rate
                    for i = 1:(noPoints-1)
                        psi_dot_temp(i+1,1) = (psi_temp(i+1)-psi_temp(i))/dt;                           % derivative of the angle
                    end
                end
            else
                psi_1    = psi_des(ii-1,:);                                                             % desired heading in wpt(ii-1)
                psi_2    = psi_des(ii,:);                                                               % desired heading in wpt(ii)
                psi_temp = zeros(noPoints,1);                                                           % initialization of psi_temp
                if psi_1 == psi_2                                                                       % if the angles are identical, no turning is performed
                    psi_temp     = psi_1*ones(noPoints,1);
                    psi_dot_temp = zeros(noPoints,1);
                else
                    for i = 1:length(psi_temp)
                        psi_temp(i) = (psi_2-psi_1)/noPoints*i+psi_1;                                    % psi reference is a straight line that connects initial and final headings
                    end
                    psi_dot_temp    = zeros(noPoints,1);                                                 % initialization of psi_dot
                    psi_dot_temp(1) = psi_dot_end;                                                       % to ensure continuity of the turning rate
                    for i = 1:(noPoints-1)
                        psi_dot_temp(i+1,1) = (psi_temp(i+1)-psi_temp(i))/dt;                            % derivative of the angle
                    end
            end
            end
            
            path(:,3) = psi_temp;
            vel_ref_temp(1:noPoints,1) = zeros(noPoints,1);
            vel_refx = [ vel_refx; vel_ref_temp ];                                                     % saving variables
            vel_refy = [ vel_refy; vel_ref_temp ];                                                     % saving variables
            pos_refx = [path(:,1)];
            pos_refy = [path(:,2)];
%             pos_refz = [pos_refz;wpt(ii,3)*ones(noPoints,1)];
            pos_refz = wpt(ii-1,3)*ones(noPoints,1);% same position in z
            psi_dot = [psi_dot;psi_dot_temp];
            total_n_steps = total_n_steps + noPoints;
            psi = [psi;psi_temp];
        
        %%% DISPLACEMENT CASE %%%
        else
            [path,points,type,dist] = dubins_curve(p1, p2, r, stepsize, quiet);                         % path is [x,y,heading]
            noPoints = length(path);                                                                    % the number of points depends on the path number of points (stepsize and length of path)
%             dtt = (ToA(ii)-t_end)/noPoints;
            t_ref(end:end+noPoints) = linspace(t_end,ToA(ii),noPoints+1);
            dt = t_ref(end)-t_ref(end-1);
            pos_refx = [path(:,1)];
            pos_refy = [path(:,2)];
            pos_refz = wpt(ii-1,3)*ones(noPoints,1);% same position in z
            % points is [x1, y1, heading, position in segment; x2, y2, heading, position in segment]
            % type is the kind of dubins curve: LSL, RSL... (see beginning of
            % code)
            %%% LINEAR VELOCITY GENERATION %%%
            % Linear velocity in x using a trapezoidal profile
            v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_x)*a));...
                (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_x)*a)) ]/2;
            v_max = min((v_max))*sign(delta_x);
            delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
            t_star = abs(v_max)/a;
            t_hat = t_star + delta_t;
            vel_ref_temp(1:noPoints,1) = v_max;
            up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
            down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
            vel_ref_temp(up) = sign(delta_x)*a*(t_ref(end-noPoints+up) - t_end);
            vel_ref_temp(down) = v_max - sign(delta_x)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
            kk = find(vel_ref_temp>v_max);
            vel_ref_temp(kk) = v_max;
            vel_refx = [ vel_refx; vel_ref_temp ];                                                     
            % Linear velocity in y using a trapezoidal profile
            v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_y)*a));...
                (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_y)*a)) ]/2;
            v_max = min((v_max))*sign(delta_y);
            delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
            t_star = abs(v_max)/a;
            t_hat = t_star + delta_t;
            vel_ref_temp(1:noPoints,1) = v_max;
            up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
            down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
            vel_ref_temp(up) = sign(delta_y)*a*(t_ref(end-noPoints+up) - t_end);
            vel_ref_temp(down) = v_max - sign(delta_y)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
            kk = find(vel_ref_temp>v_max);
            vel_ref_temp(kk) = v_max;
            vel_refy = [ vel_refy; vel_ref_temp ];                                                     
            
            % OBTAINING YAW AND YAW VELOCITY % 
            if isempty(psi_des) == 1
            %%% TURNS %%%
            % first turn
            point_1 = p1;                                                                               % first point is the initial wpt of the sgmnt
            point_2 = points(1,:);                                                                      % second point is end of first curve
            subPoints = points(1,4);
            if subPoints == 0
                psi_dot_temp = [];                                                                      % initialize the temporal storage vector for delta psi
            else
                psi_dot_temp = [psi_dot_end];                                                                     % initialize the temporal storage vector for delta psi
            end
%             if isempty(psi_des)==1
            psi_temp = path(1:subPoints,3);                                                             % asigned the third component of path from dubins_curve.m
            time_step = t_ref(end)-t_ref(end-1);
            for i=1:subPoints-1
                aa = (psi_temp(i+1)-psi_temp(i))/time_step;                                             % compute the discrete derivative of psi
                psi_dot_temp = [psi_dot_temp; aa];
            end
            psi_temp = path(:,3);
%             kk = find(abs(psi_temp(:,1)-0.7839)<0.0001);
%             if isempty(kk) == 1
%             else 
%                 a= psi_temp(kk);
%             end
            % Saturation to avoid possible extreme values
            kk = find(psi_dot_temp > 10);
            if isempty(kk)==0
                if psi_dot_temp(kk) == psi_dot_temp(end-1)
                    psi_dot_temp(kk) = psi_dot_temp(end);
                else
                    psi_dot_temp(kk) = psi_dot_temp(end-1);
                end
            end
            kk = find(psi_dot_temp < -10);
            if isempty(kk)==0
                psi_dot_temp(kk) = psi_dot_temp(end-1);
            end
            % straight segment
            psi_dot_temp = [psi_dot_temp;zeros(points(2,4)-point_2(4),1)];
            psi_dot = [psi_dot;psi_dot_temp];
            total_n_steps = total_n_steps + noPoints;
            
            
            % second turn (if any)
            if points(2,4)~=noPoints
                % In this section we compute:
                % Turning rate
                psi_dot_temp = [];                                  % initialize the temporal storage vector for delta psi
                point_1 = points(2,:);                              % first point is the initial wpt of the last segment
                point_2 = p2;                                       % second point is end of segment
                subPoints = noPoints-point_1(4);                    % the number of points depends on the subsegment number of points (stepsize and length of path)
                for i=point_1(4)+1:length(psi_temp)
                    aa = (psi_temp(i)-psi_temp(i-1))/time_step;
                    psi_dot_temp = [psi_dot_temp; aa];
                end
                % To control extreme values
                kk = find(psi_dot_temp > 10);
                if isempty(kk)==0
                    if psi_dot_temp(kk) == psi_dot_temp(end-1)
                        psi_dot_temp(kk) = psi_dot_temp(end);
                    else
                        psi_dot_temp(kk) = psi_dot_temp(end-1);
                    end
                end
                kk = find(psi_dot_temp < -10);
                if isempty(kk)==0
                    psi_dot_temp(kk) = psi_dot_temp(end-1);
                end
                psi_dot      = [psi_dot;psi_dot_temp];
            end
            else
                psi_1 = psi_des(ii-1,1);
                psi_2 = psi_des(ii,1);
                psi_temp = zeros(noPoints,1);
                psi_temp(1) = psi_end;                                                       % to ensure continuity of the turning rate
                for i = 1:noPoints
                    psi_temp(i,1) = (psi_2-psi_1)/noPoints*i+psi_1;                                    % psi reference is a straight line that connects initial and final headings
                end
                psi_dot_temp    = zeros(noPoints,1);                                                 % initialization of psi_dot
                psi_dot_temp(1) = psi_dot_end;                                                       % to ensure continuity of the turning rate
                for i = 1:(noPoints-1)
                    psi_dot_temp(i+1,1) = (psi_temp(i+1)-psi_temp(i))/dt;                            % derivative of the angle
                end
                 psi_dot = [psi_dot;psi_dot_temp];
            end           
            psi = [psi;psi_temp];
        end
        vel_refz = [ vel_oldz; zeros(noPoints,1) ]; % saving

    end
    vel_oldz = vel_refz;  
    psi_dot_end  = psi_dot_temp(end);
    psi_end = psi_temp(end);
    kk = find(psi>pi);
    psi(kk) = psi(kk)-2*pi;

    clear vel_ref_temp
    t_end = t_ref(end);                    % refresh last instant of time
    
    vel_ref = [ vel_refx, vel_refy, vel_refz ];  % put all the velocities together
    pos_ref      = [pos_ref;[pos_refx pos_refy pos_refz]];
    
    
    
    
end

%%% Make the first segment of psi and psi_dot to match with the second
if isempty(psi_des)==1
psi_2 = psi(1000+2,1);                                                                              % First non-zero value of psi (start of Bezier curve)
psi_1 = psi(1);                                                                                         % First value of psi
psi_temp = linspace(psi_1,psi_2,1000);                                                              % linear interpolation in psi
psi(1:1000,1) = psi_temp;                                                                           % substitution
psi(1000+1,1) = psi(1000,1);                                                                    % First component must not be 0
for i=2:1000                                                                                        % Derivative of psi
    psi_dot(i,1) = (psi(i)-psi(i-1))/dt;
end
else
psi_2 = psi_des(2);                                                                              % First non-zero value of psi (start of Bezier curve)
psi_1 = psi_des(1);                                                                                         % First value of psi
psi_temp = linspace(psi_1,psi_2,1000);                                                              % linear interpolation in psi
psi(1:1000,1) = psi_temp;                                                                           % substitution
psi(1000+1,1) = psi(1000,1);                                                                    % First component must not be 0
for i=2:1000                                                                                        % Derivative of psi
    psi_dot(i,1) = (psi(i)-psi(i-1))/dt;
end
end
t_ref(end)=[];
end