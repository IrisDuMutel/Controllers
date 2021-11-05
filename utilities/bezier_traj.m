function [path,path_dot] = bezier_traj(P1,P2,PC1,PC2,noPoints)

%--------------------------------------------------------------------------
% Author: Iris David Du Mutel
% Year: 2019
% -------------------------------------------------------------------------
% 
% This function computes the path created amongst two given points P1, P2
% and the corresponding control points PC1 and PC2, creating a cubic Bézier
% Curve. The step is given by noPoints.
% 
% Input:    P1      : starting point
%           P2      : ending point
%           PC1     : first control point
%           PC2     : second control point
%           noPoints: number of points in which the curve is divided
% Output:   path    : set of coordinates of the points belonging to the
%                     generated curve.
%           path_dot: set of derivatives of the Bézier curve along 't'. 
%                     First component corresponds to the derivative in x,
%                     and the second is the derivative in y.
%
%--------------------------------------------------------------------------

dist      = sqrt((P2(1)-P1(1))^2+(P2(2)-P1(2))^2);
stepsize  = dist/noPoints;
t = 0;  s = 0;
x = []; y =[];
x_dot     = [];
y_dot     = [];

for n = 1:noPoints
    x_temp    = (1-t)^3*P1(1)+3*t*(1-t)^2*PC1(1)+3*t^2*(1-t)*PC2(1)+t^3*P2(1);              % x coordinate of the Bézier curve
    xdot_temp = 3*(1-t)^2*(PC1(1)-P1(1))+6*(1-t)*t*(PC2(1)-PC1(1))+3*t^2*(P2(1)-PC2(1));    % x component of the derivative of the curve in t
    y_temp    = (1-t)^3*P1(2)+3*t*(1-t)^2*PC1(2)+3*t^2*(1-t)*PC2(2)+t^3*P2(2);              % y coordinate of the Bézier curve
    ydot_temp = 3*(1-t)^2*(PC1(2)-P1(2))+6*(1-t)*t*(PC2(2)-PC1(2))+3*t^2*(P2(2)-PC2(2));    % y component of the derivative of the curve in t
    x         = [x;x_temp];
    y         = [y;y_temp];
    x_dot     = [x_dot;xdot_temp]; 
    y_dot     = [y_dot;ydot_temp];
    s         = s + stepsize;
    t         = s/dist;
end

path     = [x,y];
path_dot = [x_dot,y_dot];

end