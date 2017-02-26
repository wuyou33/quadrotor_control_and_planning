function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  
%   If no path is found, PATH is a 0-by-3 matrix.  
%   Consecutive points in PATH should not be farther apart than neighboring cells in the map 
%   (e.g.., if 5 consecutive points in PATH are co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end

% astar = true;

path = [];
num_expanded = 0; % the number of nodes for whom you update the distance.

S = [round(start(1)/map.xy_res)-map.boundary_min(1)+1 round(start(2)/map.xy_res)-map.boundary_min(2)+1 round(start(3)/map.z_res)-map.boundary_min(3)+1];
G = [round(goal(1)/map.xy_res)-map.boundary_min(1)+1 round(goal(2)/map.xy_res)-map.boundary_min(2)+1 round(goal(3)/map.z_res)-map.boundary_min(3)+1];

% for z resolution > z size, not sure 
if map.z_res > map.boundary_z
    if S(3)>G(3)
        S(3)=2;
        G(3)=1;
    else G(3)=2;
         S(3)=1;
    end
end
Q = map.table;
g = inf(map.boundary_sz(1)*map.boundary_sz(2)*map.boundary_sz(3),1);
if astar
f = inf(map.boundary_sz(1)*map.boundary_sz(2)*map.boundary_sz(3),1);
end
p = NaN(map.boundary_sz(1)*map.boundary_sz(2)*map.boundary_sz(3),1);

g((S(3)-1)*map.boundary_sz(1)*map.boundary_sz(2)+(S(2)-1)*map.boundary_sz(1)+S(1))=0;
if astar
f((S(3)-1)*map.boundary_sz(1)*map.boundary_sz(2)+(S(2)-1)*map.boundary_sz(1)+S(1)) = sqrt((goal(1)-start(1))^2+(goal(2)-start(2))^2+(goal(3)-start(3))^2);% g0+h0
end

while Q(G(1),G(2),G(3))==0 && min(g)<inf % equals to min(f)<inf
      if astar
          [~,u] = min(f);
      else
          [~,u] = min(g);
      end
      ind_x = mod(mod(u,map.boundary_sz(1)*map.boundary_sz(2)),map.boundary_sz(1));
      ind_y = ceil(mod(u,map.boundary_sz(1)*map.boundary_sz(2))/map.boundary_sz(1));
      ind_z = ceil(u/(map.boundary_sz(1)*map.boundary_sz(2)));
      if mod(mod(u,map.boundary_sz(1)*map.boundary_sz(2)),map.boundary_sz(1)) == 0
          ind_x = map.boundary_sz(1);
      end
      if mod(u,map.boundary_sz(1)*map.boundary_sz(2))/map.boundary_sz(1) == 0
          ind_y = map.boundary_sz(2);
      end
      Q (ind_x, ind_y, ind_z)=1; 
      
      num_expanded = num_expanded + 1;

      % connectivity 3D 6-connected: x y z +-1 
      if ind_x <= map.boundary_sz(1)-1
      if Q(ind_x+1,ind_y,ind_z)==0
        d = g(u)+map.xy_res; 
        if d < g(u+1)
            g(u+1)=d;
            p(u+1)=u;
            if astar
            f(u+1)=g(u+1)+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      if ind_x>=2
      if Q(ind_x-1,ind_y,ind_z)==0
        d = g(u)+map.xy_res; 
        if d < g(u-1)
            g(u-1)=d;
            p(u-1)=u;
            if astar
            f(u-1)=g(u-1)+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      if ind_y <= map.boundary_sz(2)-1
      if Q(ind_x,ind_y+1,ind_z)==0
        d = g(u)+map.xy_res; 
        if d < g(u+map.boundary_sz(1))
            g(u+map.boundary_sz(1))=d;
            p(u+map.boundary_sz(1))=u;
            if astar
            f(u+map.boundary_sz(1))=g(u+map.boundary_sz(1))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      if ind_y>= 2
      if Q(ind_x,ind_y-1,ind_z)==0
        d = g(u)+map.xy_res; 
        if d < g(u-map.boundary_sz(1))
            g(u-map.boundary_sz(1))=d;
            p(u-map.boundary_sz(1))=u;
            if astar
            f(u-map.boundary_sz(1))=g(u-map.boundary_sz(1))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      if ind_z <= map.boundary_sz(3)-1
      if Q(ind_x,ind_y,ind_z+1)==0
        d = g(u)+map.z_res; 
        if d < g(u+map.boundary_sz(1)*map.boundary_sz(2))
            g(u+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+map.boundary_sz(1)*map.boundary_sz(2))=g(u+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      if ind_z >= 2
      if Q(ind_x,ind_y,ind_z-1)==0
        d = g(u)+map.z_res; 
        if d < g(u-map.boundary_sz(1)*map.boundary_sz(2))
            g(u-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-map.boundary_sz(1)*map.boundary_sz(2))=g(u-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      % + 4 connected xy z for phase3&4
      if ind_x <= map.boundary_sz(1)-1 && ind_y <= map.boundary_sz(2)-1
      if Q(ind_x+1,ind_y+1,ind_z)==0
        d = g(u)+map.xy_res*sqrt(2); 
        if d < g(u+1+map.boundary_sz(1))
            g(u+1+map.boundary_sz(1))=d;
            p(u+1+map.boundary_sz(1))=u;
            if astar
            f(u+1+map.boundary_sz(1))=g(u+1+map.boundary_sz(1))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x <= map.boundary_sz(1)-1 && ind_y>= 2
      if Q(ind_x+1,ind_y-1,ind_z)==0
        d = g(u)+map.xy_res*sqrt(2); 
        if d < g(u+1-map.boundary_sz(1))
            g(u+1-map.boundary_sz(1))=d;
            p(u+1-map.boundary_sz(1))=u;
            if astar
            f(u+1-map.boundary_sz(1))=g(u+1-map.boundary_sz(1))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >= 2 && ind_y <= map.boundary_sz(2)-1
      if Q(ind_x-1,ind_y+1,ind_z)==0
        d = g(u)+map.xy_res*sqrt(2); 
        if d < g(u-1+map.boundary_sz(1))
            g(u-1+map.boundary_sz(1))=d;
            p(u-1+map.boundary_sz(1))=u;
            if astar
            f(u-1+map.boundary_sz(1))=g(u-1+map.boundary_sz(1))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >= 2 && ind_y >= 2
      if Q(ind_x-1,ind_y-1,ind_z)==0
        d = g(u)+map.xy_res*sqrt(2); 
        if d < g(u-1-map.boundary_sz(1))
            g(u-1-map.boundary_sz(1))=d;
            p(u-1-map.boundary_sz(1))=u;
            if astar
            f(u-1-map.boundary_sz(1))=g(u-1-map.boundary_sz(1))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-1+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      % + 4 connected xz y for phase3&4
      if ind_x <= map.boundary_sz(1)-1 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x+1,ind_y,ind_z+1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u+1+map.boundary_sz(1)*map.boundary_sz(2))
            g(u+1+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+1+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+1+map.boundary_sz(1)*map.boundary_sz(2))=g(u+1+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x <= map.boundary_sz(1)-1 && ind_z >= 2
      if Q(ind_x+1,ind_y,ind_z-1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u+1-map.boundary_sz(1)*map.boundary_sz(2))
            g(u+1-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+1-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+1-map.boundary_sz(1)*map.boundary_sz(2))=g(u+1-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >= 2 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x-1,ind_y,ind_z+1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u-1+map.boundary_sz(1)*map.boundary_sz(2))
            g(u-1+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-1+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-1+map.boundary_sz(1)*map.boundary_sz(2))=g(u-1+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >= 2 && ind_z >= 2
      if Q(ind_x-1,ind_y,ind_z-1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u-1-map.boundary_sz(1)*map.boundary_sz(2))
            g(u-1-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-1-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-1-map.boundary_sz(1)*map.boundary_sz(2))=g(u-1-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-1+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      % + 4 connected yz x for phase3&4
      if ind_y <= map.boundary_sz(2)-1 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x,ind_y+1,ind_z+1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))
            g(u+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=g(u+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_y <= map.boundary_sz(2)-1 && ind_z >= 2
      if Q(ind_x,ind_y+1,ind_z-1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))
            g(u+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=g(u+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_y >= 2 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x,ind_y-1,ind_z+1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))
            g(u-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=g(u-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_y >= 2 && ind_z >= 2
      if Q(ind_x,ind_y-1,ind_z-1)==0
        d = g(u)+hypot(map.xy_res,map.z_res); 
        if d < g(u-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))
            g(u-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=g(u-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-1+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      % + 8 connected xyz for phase3&4
      if ind_x <= map.boundary_sz(1)-1 && ind_y <= map.boundary_sz(2)-1 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x+1,ind_y+1,ind_z+1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u+1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))
            g(u+1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=g(u+1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x <= map.boundary_sz(1)-1 && ind_y <= map.boundary_sz(2)-1 && ind_z >= 2
      if Q(ind_x+1,ind_y+1,ind_z-1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u+1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))
            g(u+1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=g(u+1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x <= map.boundary_sz(1)-1 && ind_y >=2 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x+1,ind_y-1,ind_z+1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u+1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))
            g(u+1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=g(u+1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x <= map.boundary_sz(1)-1 && ind_y >=2 && ind_z >=2
      if Q(ind_x+1,ind_y-1,ind_z-1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u+1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))
            g(u+1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u+1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u+1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=g(u+1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >=2 && ind_y <= map.boundary_sz(2)-1 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x-1,ind_y+1,ind_z+1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u-1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))
            g(u-1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=g(u-1+map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >=2 && ind_y <= map.boundary_sz(2)-1 && ind_z >= 2
      if Q(ind_x-1,ind_y+1,ind_z-1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u-1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))
            g(u-1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=g(u-1+map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >=2 && ind_y >=2 && ind_z <= map.boundary_sz(3)-1
      if Q(ind_x-1,ind_y-1,ind_z+1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u-1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))
            g(u-1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))=g(u-1-map.boundary_sz(1)+map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if ind_x >=2 && ind_y >=2 && ind_z >=2
      if Q(ind_x-1,ind_y-1,ind_z-1)==0
        d = g(u)+hypot(map.xy_res*sqrt(2),map.z_res); 
        if d < g(u-1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))
            g(u-1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=d;
            p(u-1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=u;
            if astar
            f(u-1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))=g(u-1-map.boundary_sz(1)-map.boundary_sz(1)*map.boundary_sz(2))+sqrt(((ind_x-2+map.boundary_min(1))*map.xy_res-goal(1))^2+((ind_y-2+map.boundary_min(2))*map.xy_res-goal(2))^2+((ind_z-2+map.boundary_min(3))*map.z_res-goal(3))^2);
            end
        end
      end
      end
      
      if astar
          f(u)=inf;
      else
          g(u)=inf;
      end
end

num_expanded = num_expanded + 1;% counting should include goal

child = (G(3)-1)*map.boundary_sz(1)*map.boundary_sz(2)+(G(2)-1)*map.boundary_sz(1)+G(1);
parent = p(child);
path_num = child;
while parent ~= (S(3)-1)*map.boundary_sz(1)*map.boundary_sz(2)+(S(2)-1)*map.boundary_sz(1)+S(1)
      parent = p(child);
      path_num = [parent;path_num];
      child = parent;
end
      
path = start;
for i = 2:size(path_num,1)-1
      point_num = path_num(i);
      point_x = (mod(mod(point_num,map.boundary_sz(1)*map.boundary_sz(2)),map.boundary_sz(1))-1+map.boundary_min(1))*map.xy_res;
      point_y = (ceil(mod(point_num,map.boundary_sz(1)*map.boundary_sz(2))/map.boundary_sz(1))-1+map.boundary_min(2))*map.xy_res;
      point_z = (ceil(point_num/(map.boundary_sz(1)*map.boundary_sz(2)))-1+map.boundary_min(3))*map.z_res;
      if mod(mod(point_num,map.boundary_sz(1)*map.boundary_sz(2)),map.boundary_sz(1)) == 0
          point_x = (map.boundary_sz(1)-1)*map.xy_res;
      end
      if mod(point_num,map.boundary_sz(1)*map.boundary_sz(2))/map.boundary_sz(1) == 0
          point_y = (map.boundary_sz(2)-1)*map.xy_res;
      end
      point = [point_x, point_y, point_z];
      path = [path;point];
end
path = [path;goal];


end
