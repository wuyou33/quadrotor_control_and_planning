function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% number of points
M = size(points,1);
C = zeros(M,1);

for i = 1:M
    x = ceil(points(i,1)/map.xy_res-map.boundary_min(1)+1);
    y = ceil(points(i,2)/map.xy_res-map.boundary_min(2)+1);
    z = ceil(points(i,3)/map.z_res-map.boundary_min(3)+1);
    if x>map.boundary_sz(1) || x<1 || y>map.boundary_sz(2) ||y<1 ||z>map.boundary_sz(3)  ||z<1
    C(i) = 1; % out of boundary
    else
    C(i) = map.table(x,y,z);
    end
end

end
