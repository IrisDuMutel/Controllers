function [pos_ref,vel_ref,t_ref,trn_pts,delta_psi] = trajplann5(wpt,ToA,theta)
% trajplanner5 contains the path corresponding to that composed of splines

a = 9.81/2;
t_ref = [];
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
trn_pts  = [];
delta_psi = [];
delta_psi_temp = [];
t_end = 0;
n = size(wpt,1);



for ii = 2:n
    noPoints = 102;                                                             % default
    % In which direction are we moving?
    delta_x = wpt(ii,1) - wpt(ii-1,1);                                          % displacement in x-axis
    delta_y = wpt(ii,2) - wpt(ii-1,2);                                          % displacement in y-axis
    delta_z = wpt(ii,3) - wpt(ii-1,3);                                          % displacement in z-axis
    
    if delta_z ~= 0
        % as usual
        t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);           % create time vector with noPoints elements between t_end and ToA
        dt = t_ref(2+noPoints*(ii-2)) - t_ref(1+noPoints*(ii-2));               % time step in vector t_ref
        v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a));...
            (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a)) ]/2;
        v_max = min(v_max)*sign(delta_z);
        delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
        t_star = abs(v_max)/a;
        t_hat = t_star + delta_t;
        vel_ref_temp(1:noPoints,1) = v_max;
        up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
        down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
        vel_ref_temp(up) = sign(delta_z)*a*(t_ref(end-noPoints+up) - t_end);
        vel_ref_temp(down) = v_max - sign(delta_z)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
        
        if isempty(pos_refz)
            pos_refz(1) = 0;                 % if it's empty, first component is 0
        else
            pos_refz(end+1) = pos_refz(end); % else, copy last component
        end
        for jj = 2:noPoints                  % for the segment being analyzed
            pos_refz(end+1) = (pos_refz(end)) + ...
                (vel_ref_temp(jj) + vel_ref_temp(jj-1))/2*dt;   % compute new positions integrating all along the segment(trapezoidal method)
        end
        vel_refx = [vel_refx; zeros(noPoints,1)];
        vel_refy = [vel_refy; zeros(noPoints,1)];   
        pos_x(end+1:end+noPoints) = wpt(ii-1,1)*ones(noPoints,1);% same position in x
        pos_y(end+1:end+noPoints) = wpt(ii-1,2)*ones(noPoints,1);% same position in x
        pos_refx = [pos_refx;pos_x];
        pos_refy = [pos_refy;pos_y];
        delta_psi = [delta_psi;zeros(length(vel_ref_temp),1)];   % when going up or down, no yaw 
        
    else % this means that the movement is happening in the x-y plane
        if delta_x == 0
            

            
            p1   = [wpt(ii-1,1:2),theta(ii-1)];         % starting point (of the segment)
            p2   = [wpt(ii,1:2),theta(ii)];             % ending point (of the segment)
            x    = [p1(1,1) p2(1,1)];
            y    = [p1(1,2) p2(1,2)];
            points = [x;y];
            rot_mat = [cos(theta(ii-1)/2) sin(theta(ii-1)/2)
                        -sin(theta(ii-1)/2) cos(theta(ii-1)/2)];
            points = rot_mat*points;
            x = points(1,:); y = points(2,:);
            cs = spline(x,[(theta(ii-1)-theta(ii-1)/4) y (theta(ii)-theta(ii-1)/4)]);
            xx = linspace(x(1),x(2),noPoints);
            yy = ppval(cs,xx);
            path = [xx' yy'];
            dp = ppder(cs); % coeficients of the dy/dx
            long = 0;
            for i=1:(length(path)-1)
                long = long + sqrt((path(i,1)-path(i+1,1))^2+(path(i,2)-path(i+1,2))^2); % computing the length of the trajectory step by step
            end
            dydx = ppval(dp,xx); %values of the derivative in the points in xx
            theta_temp = atan2(dydx,1);
%             figure()
%             plot(x,y,'o',xx,yy,'-');
%             figure()
%             plot(xx,theta_temp)
            dist_xy = long;
            noPoints = length(xx);                    % the number of points depends on the path number of points (stepsize and length of path)
            t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);
            
            % Linear velocity modulus using a trapezoidal profile
            v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(dist_xy)*a));...
                (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(dist_xy)*a)) ]/2;
            v_max = min(abs(v_max))*sign(dist_xy);
            delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
            t_star = abs(v_max)/a;
            t_hat = t_star + delta_t;
            %     t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);
            vel_ref_temp(1:noPoints,1) = v_max;
            up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
            down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
            vel_ref_temp(up) = sign(dist_xy)*a*(t_ref(end-noPoints+up) - t_end);
            vel_ref_temp(down) = v_max - sign(dist_xy)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
            
            % obtaining NED components of the velocity depending on the heading
            vel_refx_temp = vel_ref_temp;
            vel_refy_temp = vel_ref_temp;
            % what is my heading now?
            vel_refx_temp = vel_refx_temp.*cos(theta_temp');
            vel_refy_temp = vel_refy_temp.*sin(theta_temp');
            
            for i=1:noPoints
%                 vel_refx_rot = rot_mat'*vel_refx_temp(i:i+1,1);  % saving variables
%                 vel_refy_rot = rot_mat'*vel_refy_temp(i:i+1,1);  % saving variables
                pos_temp    = rot_mat'*path(i,:)';
%                 posy_temp    = rot_mat'*path(i,2);
                
                pos_refx = [pos_refx,pos_temp(1)];
                pos_refy = [pos_refy,pos_temp(2)];
            end
            vel_refx_rot = vel_refx_temp.*sin(theta(ii-1)/2)+vel_refy_temp.*sin(theta(ii-1)/2);
            vel_refy_rot = vel_refx_temp.*cos(theta(ii-1)/2)+vel_refy_temp.*cos(theta(ii-1)/2);
            vel_refx = [ vel_refx; vel_refx_rot ];
            vel_refy = [ vel_refy; vel_refy_rot ];
            %             vel_refx = [ vel_refx; vel_refx_temp ];  % saving variables
%             vel_refy = [ vel_refy; vel_refy_temp ];  % saving variables
            clear vel_ref_temp
            
            %%% TURNS %%%
            delta_psi_temp = [];                       % initialize the temporal storage vector for delta psi
            time_step = t_ref(2)-t_ref(1);
            for i=1:(length(path)-1)
                a = (theta_temp(i+1)-theta_temp(i)-2*theta(ii-1))/time_step;
                delta_psi_temp = [delta_psi_temp; a];
            end
            delta_psi = [delta_psi;delta_psi_temp];
            vel_ref_temp = zeros(noPoints,1);                           % no mvmt in x, so zeros.
%             pos_refx = [pos_refx,rot_mat'*path(:,1)'];
%             pos_refy = [pos_refy,rot_mat'*path(:,2)'];
            pos_refz(end+1:end+noPoints) = wpt(ii-1,3)*ones(noPoints,1);% same position in x
            vel_ref_temp = zeros(noPoints,1);
            clear theta_temp
        
        
        
        
        
        
        
        
        
        
        
        else
            p1   = [wpt(ii-1,1:2),theta(ii-1)];         % starting point (of the segment)
            p2   = [wpt(ii,1:2),theta(ii)];             % ending point (of the segment)
            x    = [p1(1,1) p2(1,1)];
            y    = [p1(1,2) p2(1,2)];
            cs = spline(x,[theta(ii-1) y theta(ii)]);
            xx = linspace(x(1),x(2),101);
            yy = ppval(cs,xx);
            path = [xx' yy'];
            dp = ppder(cs); % coeficients of the dy/dx
            long = 0;
            for i=1:(length(path)-1)
                long = long + sqrt((path(i,1)-path(i+1,1))^2+(path(i,2)-path(i+1,2))^2);
            end
            dydx = ppval(dp,xx); %values of the derivative in the points in xx
            theta_temp = atan2(dydx,1);
%             figure()
%             plot(x,y,'o',xx,yy,'-');
%             figure()
%             plot(xx,theta_temp)
            dist_xy = long;
            noPoints = length(xx);                    % the number of points depends on the path number of points (stepsize and length of path)
            t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);
            
            % Linear velocity modulus using a trapezoidal profile
            v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(dist_xy)*a));...
                (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(dist_xy)*a)) ]/2;
            v_max = min(v_max)*sign(dist_xy);
            delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
            t_star = abs(v_max)/a;
            t_hat = t_star + delta_t;
            vel_ref_temp(1:noPoints,1) = v_max;
            up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
            down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
            vel_ref_temp(up) = sign(dist_xy)*a*(t_ref(end-noPoints+up) - t_end);
            vel_ref_temp(down) = v_max - sign(dist_xy)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
            
            % obtaining NED components of the velocity depending on the heading
            vel_refx_temp = vel_ref_temp;
            vel_refy_temp = vel_ref_temp;
            % what is my heading now?
            vel_refx_temp = vel_refx_temp.*cos(theta_temp');
            vel_refy_temp = vel_refy_temp.*sin(theta_temp');
            vel_refx = [ vel_refx; vel_refx_temp ];  % saving variables
            vel_refy = [ vel_refy; vel_refy_temp ];  % saving variables
            clear vel_ref_temp
            
            %%% TURNS %%%
            delta_psi_temp = [];                       % initialize the temporal storage vector for delta psi
            time_step = t_ref(2)-t_ref(1);
            for i=1:(length(path)-1)
                aa = (theta_temp(i+1)-theta_temp(i))/time_step;
                delta_psi_temp = [delta_psi_temp; aa];
            end
            delta_psi = [delta_psi;delta_psi_temp];
            vel_ref_temp = zeros(noPoints,1);                           % no mvmt in x, so zeros.
        pos_refx = [pos_refx,path(:,1)'];
        pos_refy = [pos_refy,path(:,2)'];
        pos_refz(end+1:end+noPoints) = wpt(ii-1,3)*ones(noPoints,1);% same position in x
        clear theta_temp

        end
        
    end

    vel_refz = [ vel_oldz; vel_ref_temp ]; % saving
    vel_oldz = vel_refz;                   % replacing
    clear vel_ref_temp
    
    t_end = t_ref(end);                    % refresh last instant of time
    
    end

    vel_ref = [ vel_refx, vel_refy, vel_refz ];  % put all the velocities together
    pos_ref = [ pos_refx' pos_refy' pos_refz' ]; % put all the positions together




end