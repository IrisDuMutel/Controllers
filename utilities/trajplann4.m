function [pos_ref,vel_ref,t_ref,psi,psi_dot] = trajplann4(wpt,ToA,theta)
% trajplanner4 contains the path corresponding to that of dubins curves.
% the velocity profile depends on the movement taking place. When the
% displacement happens along the z-axis, it's a trapezoidal velocity
% profile depending on the distance being flown.
% If the displacement is in the x-y plane, then a trapezoidal velocity
% profile is used as well counting the distance of the dubins curve between
% two consecutive wpts.
% The angular velocity for yaw is considered a special case in which a
% maximum is set and then, a regular trapezoid is

a = 9.81/2;
t_ref = [];
psi = [];
psi_temp = [];
vel_refx = [];
vel_refy = [];
vel_refz = [];
pos_refx = [];
pos_refy = [];
pos_refz = [];
vel_oldx = [];
vel_oldy = [];
vel_oldz = [];
pos_x    = [];
pos_y    = [];
psi_dot = [];
psi_dot_end  = 0;
psi_dot_temp = [];
t_ref_turn1 = [];
t_end = 0;
noPoints = 1000;
stepsize = 1/noPoints;
n = size(wpt,1);
r = 0.2;         % curvature radius (meters)
quiet = true;
total_n_steps = 0;

% there are 6 types of dubin's curve, only one will have minimum cost
% LSL = 1;
% LSR = 2;
% RSL = 3;
% RSR = 4;
% RLR = 5;
% LRL = 6;
% 
% %     The three segment types a path can be made up of
% L_SEG = 1;
% S_SEG = 2;
% R_SEG = 3;
% %     The segment types for each of the Path types
% 
% DIRDATA = [ L_SEG, S_SEG, L_SEG ;...
%     L_SEG, S_SEG, R_SEG ;...
%     R_SEG, S_SEG, L_SEG ;...
%     R_SEG, S_SEG, R_SEG ;...
%     R_SEG, L_SEG, R_SEG ;...
%     L_SEG, R_SEG, L_SEG ];


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
        t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);                               % create time vector with noPoints elements between t_end and ToA
        dt = t_ref(2+noPoints*(ii-2)) - t_ref(1+noPoints*(ii-2));                                   % time step in vector t_ref
        v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a));...
            (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a)) ]/2;
        v_max = min(v_max)*sign(delta_z);
        delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
        t_star  = abs(v_max)/a;
        t_hat   = t_star + delta_t;
        vel_ref_temp(1:noPoints,1) = v_max;
        up   = find(t_ref(end-noPoints+1:end) - t_end < t_star);
        down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
        vel_ref_temp(up)   = sign(delta_z)*a*(t_ref(end-noPoints+up) - t_end);
        vel_ref_temp(down) = v_max - sign(delta_z)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
        
        %%% CREATION OF THE POSITION VECTOR IN Z THROUGH INTEGRATION %%%
        if isempty(pos_refz)
            pos_refz(1,1) = 0;                                                                        % if it's empty, first component is 0
        else
            pos_refz(end+1,1) = pos_refz(end);                                                        % else, copy last component
        end
        for jj = 2:noPoints                                                                         % for the segment being analyzed
            pos_refz(end+1,1) = (pos_refz(end)) + ...
                (vel_ref_temp(jj) + vel_ref_temp(jj-1))/2*dt;                                       % compute new positions integrating all along the segment(trapezoidal method)
        end
        
        %%% GENERATION OF PSI AND PSI_DOT WHILE MOVING IN Z %%%
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
        
        %%% SAVING VARIABLES %%%
        vel_refx = [vel_refx; zeros(noPoints,1)];
        vel_refy = [vel_refy; zeros(noPoints,1)];
        vel_refz = [vel_refz; vel_ref_temp];
%         vel_oldz = vel_refz;
        pos_x(end+1:end+noPoints,1) = wpt(ii-1,1)*ones(noPoints,1);                                   % same position in x
        pos_y(end+1:end+noPoints,1) = wpt(ii-1,2)*ones(noPoints,1);                                   % same position in y
        pos_refx = [pos_refx;pos_x(end-noPoints+1:end,1)];
        pos_refy = [pos_refy;pos_y(end-noPoints+1:end,1)];
        total_n_steps = total_n_steps + noPoints;
        psi_dot = [psi_dot;psi_dot_temp];                                                           % when going up or down, no yaw
        psi = [psi;psi_temp];
        
    %%%% MOVEMENT IN THE X-Y PLANE %%%%
    % It can be one of two possibilities: hovering (and turning) or a linear
    % displacement following a Dubins curve
    else                                                                                            % this means that the movement is happening in the x-y plane
        p1   = [wpt(ii-1,1:2),theta(ii-1)];                                                         % starting point (of the segment)
        p2   = [wpt(ii,1:2),theta(ii)];                                                             % ending point (of the segment)
        
        %%% HOVERING CASE %%%
        if p1(1)== p2(1) && p1(2) == p2(2)                                                          % not moving but hovering
            path = [wpt(ii-1,1)*ones(noPoints,1) wpt(ii-1,2)*ones(noPoints,1)];                     % x and y are the same as in the previous wpt
            psi_1    = theta(ii-1,:);                                                               % heading in wpt(ii-1)
            psi_2    = theta(ii,:);                                                                 % heading in wpt(ii)
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
            path(:,3) = psi_temp;
            vel_ref_temp(1:noPoints,1) = zeros(noPoints,1);
            vel_refx = [ vel_refx; vel_ref_temp ];                                                     % saving variables
            vel_refy = [ vel_refy; vel_ref_temp ];                                                     % saving variables
            pos_refx = [pos_refx;path(:,1)];
            pos_refy = [pos_refy;path(:,2)];
            pos_refz = [pos_refz;wpt(ii,3)*ones(noPoints,1)];
            pos_refz(end+1:end+noPoints,1) = wpt(ii-1,3)*ones(noPoints,1);% same position in z
            psi_dot = [psi_dot;psi_dot_temp];
            total_n_steps = total_n_steps + noPoints;
            psi = [psi;psi_temp];
        
        %%% DISPLACEMENT CASE %%%
        else
            [path,points,type,dist] = dubins_curve(p1, p2, r, stepsize, quiet);                         % path is [x,y,heading]
            dist_xy = sum(dist)*r;
            % points is [x1, y1, heading, position in segment; x2, y2, heading, position in segment]
            % type is the kind of dubins curve: LSL, RSL... (see beginning of
            % code)
            %%% LINEAR VELOCITY GENERATION %%%
            noPoints = length(path);                                                                    % the number of points depends on the path number of points (stepsize and length of path)
            t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);
            % Linear velocity modulus using a trapezoidal profile (body axes)
            v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(dist_xy)*a));...
                (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(dist_xy)*a)) ]/2;
            v_max = min((v_max))*sign(dist_xy);
            delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
            t_star = abs(v_max)/a;
            t_hat = t_star + delta_t;
            vel_ref_temp(1:noPoints,1) = v_max;
            up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
            down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
            vel_ref_temp(up) = sign(dist_xy)*a*(t_ref(end-noPoints+up) - t_end);
            vel_ref_temp(down) = v_max - sign(dist_xy)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
            kk = find(vel_ref_temp>v_max);
            vel_ref_temp(kk) = v_max;
            
            
            % obtaining NED components of the velocity depending on the heading
            % it must be porjected over NED axes
            vel_refx_temp = vel_ref_temp.*cos(path(:,3));
            vel_refy_temp = vel_ref_temp.*sin(path(:,3));
            %         plot(t_ref(end+1-noPoints:end),cos(path(:,3)))
            
            vel_refx = [ vel_refx; vel_refx_temp ];                                                     % saving variables
            vel_refy = [ vel_refy; vel_refy_temp ];                                                     % saving variables
            
            %%% TURNS %%%
            % first turn
            point_1 = p1;                                                                               % first point is the initial wpt of the sgmnt
            point_2 = points(1,:);                                                                      % second point is end of first curve
            subPoints = points(1,4);
            if subPoints == 0
                psi_dot_temp = [];                                                                      % initialize the temporal storage vector for delta psi
            else
                psi_dot_temp = [0];                                                                     % initialize the temporal storage vector for delta psi
            end
            psi_temp = path(1:subPoints,3);                                                             % asigned the third component of path from dubins_curve.m
            dist_xy = dist(1)*r;
            time_step = t_ref(end)-t_ref(end-1);
            for i=1:subPoints-1
                aa = (psi_temp(i+1)-psi_temp(i))/time_step;                                             % compute the discrete derivative of psi
                psi_dot_temp = [psi_dot_temp; aa];
            end
            psi_temp = path(:,3);
            
            % then, the straight part has turning rate equal to zero
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
            
            psi_dot_temp = [psi_dot_temp;zeros(points(2,4)-point_2(4),1)];
            pos_refx = [pos_refx;path(:,1)];
            pos_refy = [pos_refy;path(:,2)];
            pos_refz(end+1:end+noPoints,1) = wpt(ii-1,3)*ones(noPoints,1);% same position in z
            psi_dot = [psi_dot;psi_dot_temp];
            total_n_steps = total_n_steps + noPoints;
            psi = [psi;psi_temp];
            
            % second turn (if any)
            if points(2,4)~=noPoints
                % In this section we compute:
                % Turning rate
                % Make a regular ending for the velocity profile
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
                
                %             psi(end-subPoints+1:end) = [psi(end-subPoints-1)*ones(subPoints,1)];
                %             to use a trapezoidal velocity profile, comment the next four
                %             lines
                vel_refx_temp = vel_refx(end-subPoints-1)*ones(subPoints,1);
                vel_refy_temp = vel_refy(end-subPoints-1)*ones(subPoints,1);
                vel_refx(end-subPoints+1:end) = vel_refx_temp;
                vel_refy(end-subPoints+1:end) = vel_refy_temp;
            end
        end


    vel_refz = [ vel_oldz; zeros(noPoints,1) ]; % saving
    end
    
    vel_oldz = vel_refz;                   % replacing
    psi_dot_end  = psi_dot_temp(end);
    kk = find(psi>pi);
    psi(kk) = psi(kk)-2*pi;
    clear vel_ref_temp
    t_end = t_ref(end);                    % refresh last instant of time
    
    vel_ref = [ vel_refx, vel_refy, vel_refz ];  % put all the velocities together
    pos_ref = [ pos_refx pos_refy pos_refz ]; % put all the positions together
    
    
    
    
end