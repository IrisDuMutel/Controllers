function [pos_ref, vel_ref, t_ref, psi, psi_dot] = trajplann6(wpt, ToA, PC, psi_des)

%--------------------------------------------------------------------------
% Author: Iris David Du Mutel
% Year: 2019
% -------------------------------------------------------------------------
% This trajectoy planner works as follows:
        % The path is computed using cubic bezier curves. To do so, the
        % waipoints and the corresponding control points are required.
        % Also, a desired heading can be introduced. If not, leave empty.
        
        % It is allowed to perform vertical turns, hovering and hovering and
        % turning.
        % If a certain psi_des is given, the yaw prfile is computed as a
        % straight line.
        
        % The linear velocities i.e. u,v and w NED are computed differently
        % from each other. w_NED is computed using a trapezoidal profile,
        % while u and v are computed using the time and position vectors.
        % It must be noted that positions and velocities on this code are 
        % computed in NED reference frame. 
        
        % The angular velocity (psi_dot) is computed derivating psi on
        % time.

% Input:    wpt     : list of waypoints of the pattern
%           ToA     : list of times of arrival at the different waypoint of
%                     the trajectory.
%           PC      : Matrix containing the control points (two control
%                     points per segment).
%           psi_des : Desired heading at every waypoint in wpt.
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
vel_ref  = [];pos_ref  = [];
a        = 9.81/2;
t_end    = 0; t_ref    = [];
vel_refx = [0];vel_refy = [0];vel_refz = [0];
pos_refx = [];pos_refy = [];pos_refz = [];
pos_x    = [];pos_y    = [];pos_z    = [];
psi_temp = [];psi      = [];
dx_end   = 0; dy_end   = 0;
psi_dot  = [];psi_dot_end  = 0; psi_dot_temp = [];

for ii = 2:length(wpt)
    noPoints = 1000;                                                                                     % number of points per segment
    delta_x = wpt(ii,1) - wpt(ii-1,1);                                                                   % displacement in x-axis
    delta_y = wpt(ii,2) - wpt(ii-1,2);                                                                   % displacement in y-axis
    delta_z = wpt(ii,3) - wpt(ii-1,3);                                                                   % displacement in z-axis
    
    %%%% MOVEMENT IN THE Z AXIS %%%%
    if delta_z ~= 0
        
        t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);                                    % create time vector with noPoints elements between t_end and ToA
        dt      = t_ref(2+noPoints*(ii-2)) - t_ref(1+noPoints*(ii-2));                                   % time step in vector t_ref
        
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
                
        %%%% CREATION OF POSITION FROM INTEGRATION OF THE VELOCITY %%%%
        
        if isempty(pos_ref)                                                                              % check the z position vector
            pos_refz(1) = 0;                                                                             % if it's empty, first component is 0
        else
            pos_refz(1) = pos_ref(end,3);                                                                % else, copy last component to provide continuity
        end
        
        for jj = 2:noPoints                                                                              % for the segment being analyzed
            pos_refz(jj,1) = (pos_refz(jj-1,1)) + (vel_ref_temp(jj) + vel_ref_temp(jj-1))/2*dt;          % compute new positions integrating all along the segment(trapezoidal method of integration)
        end
        
        %%%% CREATION OF YAW ANGLE AND VELOCITY PROFILES %%%%
        
        if isempty(psi_des)                                                                              % if no psi_des vector is desired
            psi_temp = zeros(noPoints,1);                                                                % no turning is considered, then 0
            psi_dot_temp    = zeros(noPoints,1);                                                         % initialization of psi_dot
        else
            psi_1    = psi_des(ii-1,:);                                                                  % heading in wpt(ii-1)
            psi_2    = psi_des(ii,:);                                                                    % heading in wpt(ii)
            psi_temp = zeros(noPoints,1);                                                                % initialization of psi_temp
            if psi_1 == psi_2                                                                            % if the angles are identical, no turning is performed
                psi_temp     = psi_1*ones(noPoints,1);
                psi_dot_temp = zeros(noPoints,1);
            else
                for i = 1:length(psi_temp)
                    psi_temp(i) = (psi_2-psi_1)/noPoints*i+psi_1;                                        % psi reference is a straight line that connects initial and final headings
                end
                psi_dot_temp    = zeros(noPoints,1);                                                     % initialization of psi_dot
                psi_dot_temp(1) = psi_dot_end;                                                           % to ensure continuity of the turning rate
                for i = 1:(noPoints-1)
                    psi_dot_temp(i+1,1) = (psi_temp(i+1)-psi_temp(i))/dt;                                % derivative of the angle
                end
            end
        end
        
        %%%% SAVING GENERATED VALUES %%%% 
        vel_refx = [zeros(noPoints,1)];                                                                  % no velocity in x is considered when flying vertically
        vel_refy = [zeros(noPoints,1)];                                                                  % no velocity in y is considered when flying vertically
        vel_refz = [vel_ref_temp];                                                                       % the velocity in z is saved in vel_refz 
        pos_refx = wpt(ii-1,1)*ones(noPoints,1);                                                         % no variation of x position
        pos_refy = wpt(ii-1,2)*ones(noPoints,1);                                                         % no variation of y position
        
    %%%% MOVEMENT IN THE X-Y PLANE %%%% 
    % It can be one of two possibilities: hovering (and turning) or a linear
    % displacement following a Bézier curve
    else
        t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);                                    % create time vector with noPoints elements between t_end and ToA
        dt       = t_ref(end)-t_ref(end-1);                                                              % timestep
        P1       = wpt(ii-1,:);                                                                          % coordinates of the starting point
        P2       = wpt(ii,:);                                                                            % coordinates of the ending point
        
        %%% HOVERING CASE %%%
        if P1(1)== P2(1) && P1(2) == P2(2)                                                               % not moving but hovering
            path = [wpt(ii-1,1)*ones(noPoints,1) wpt(ii-1,2)*ones(noPoints,1)];                          % x and y are the same as in the previous wpt
            path_dot = [zeros(noPoints,1) zeros(noPoints,1)];                                            % no variation in position, derivative is 0
            if isempty(psi_des)                                                                          % if no psi_des vector is desired
                psi_temp = zeros(noPoints,1);                                                            % no turning is considered, then 0
                psi_dot_temp    = zeros(noPoints,1);                                                     % initialization of psi_dot
            else
                psi_1    = psi_des(ii-1,:);                                                              % heading in wpt(ii-1)
                psi_2    = psi_des(ii,:);                                                                % heading in wpt(ii)
                psi_temp = zeros(noPoints,1);                                                            % initialization of psi_temp
                if psi_1 == psi_2                                                                        % if the angles are identical, no turning is performed
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
            %%% DISPLACEMENT CASE %%%
        else
            PC1      = PC(1:2,ii-1)';                                                                    % first control point of the segment
            PC2      = PC(3:4,ii-1)';                                                                    % second control point of the segment
            [path,path_dot] = bezier_traj(P1,P2,PC1,PC2,noPoints);                                       % compute the Bézier curve between the two wpts
            
            %%%% CREATION OF THE YAW ANGLE AND VELOCITY %%%%
            if isempty(psi_des)
                dx     = path_dot(:,1);                                                                  % derivative of the curve in x
                dy     = path_dot(:,2);                                                                  % derivative of the curve in y
                dx(1)  = dx_end;                                                                         % to ensure continuity in the angle
                dy(1)  = dy_end;                                                                         % to ensure continuity in the angle
                dx_end = dx(end);
                dy_end = dy(end);
                psi_temp = atan2(dy,dx);                                                                 % the axis is pointing downwards
                psi_temp = unwrap(psi_temp);                                                             % unwrapping psi from 0 to 2pi
                psi_dot_temp(1,1) = (psi_temp(1)-psi_dot_end)/dt;                                        % first component of the psi_dot vector
                for i = 2:(length(path)-1)
                    psi_dot_temp(i+1,1) = (psi_temp(i+1)-psi_temp(i))/dt;                                % derivative of the angle at every point
                end
                if psi_dot_temp(2) == 0
%                     psi_dot_temp(2) = psi_temp(end);
                end
            else
                psi_1    = psi_des(ii-1,:);                                                              % heading in wpt(ii-1)
                psi_2    = psi_des(ii,:);                                                                % heading in wpt(ii)
                psi_temp = zeros(noPoints,1);                                                            % initialization of psi_temp
                if psi_1 == psi_2                                                                        % if the angles are identical, no turning is performed
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
        end
        
        %%%% CREATION OF THE VELOCITY PROFILES %%%% (for both cases)
        vel_refx = vel_refx(end)*ones(noPoints,1);                                                       % initialization of the velocity vectors
        vel_refy = vel_refy(end)*ones(noPoints,1);
        vel_refz = vel_refz(end)*ones(noPoints,1);
        
        for i=1:length(path)-1
            vel_refx(i+1) = (path(i+1,1)-path(i,1))/dt;                                                  % velocity needed to move between two points in the time given
            vel_refy(i+1) = (path(i+1,2)-path(i,2))/dt;                                                  % velocity needed to move between two points in the time given
        end
        
        pos_refx = path(:,1);                                                                            % assign x coordinates to pos_refx
        pos_refy = path(:,2);                                                                            % assign y coordinates to pos_refy
        pos_refz = wpt(ii-1,3)*ones(noPoints,1);                                                         % no mvmnt in the z axis
        psi_dot_temp(1) = psi_dot_temp(2);                                                               % to avoid high values produced through derivation
        
    end
    psi          = [psi;psi_temp];
    psi_dot      = [psi_dot; psi_dot_temp];
    pos_ref      = [pos_ref;[pos_refx pos_refy pos_refz]];
    vel_ref      = [vel_ref;[vel_refx vel_refy vel_refz]];
    t_end        = t_ref(end);
    psi_dot_end  = psi_dot_temp(end);
   
    clear vel_ref_temp
    clear theta_temp
    clear psi_dot_temp
    
end
% If a constant turning rate and a linear variation of the heading angle is
% desired, uncomment this section:
%{
% Counterclockwise
psi     = linspace(0,pi/2,noPoints*(length(wpt)-2));
psi_dot = 0.2513*ones(1,length(pos_ref)-noPoints);

%{
 %parabolic angle profile and straight line turning velocity
% % to 0.5 rad
x = [1:1:1000];
y=-x.^2/500000+x./500;
psi = y;
yy = -2*x./500000 + 1/500;
psi_dot = yy;
% % % to 0.7 rad
x = [1:1:1000];
y=-7*x.^2/2500000+7*x./2500;
psi = y;
yy = -14*x./2500000 + 7/2500;
psi_dot = yy;
% % % to 0.6 rad
% x = [1:1:1000];
% y=-3*x.^2/1250000+3*x./1250;
% psi = y;
% yy = -6*x./1250000 + 3/1250;
% psi_dot = yy;
%}
psi_dot = [zeros(noPoints,1); psi_dot'];
% psi_dot = [zeros(noPoints,1); 0.2513*ones(length(pos_ref)-noPoints,1)];
psi     = [zeros(noPoints,1); psi'];
% when using constant turning rate and constant varying angle, one should
% take into account that the NED velocity is changing as well.


%}
%{
% Clockwise
psi     = linspace(pi/2,3*pi/2,noPoints*4);
psi_dot = [zeros(noPoints,1); 0.2513*ones(length(pos_ref)-noPoints,1)];
psi     = [zeros(noPoints,1); psi'];
%}

end