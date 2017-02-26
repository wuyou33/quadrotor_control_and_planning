function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

tic
clf

line=size(map.block,1);
for i = 1 : line
        x = linspace(map.block(i,1)*map.xy_res-map.xy_res, map.block(i,4)*map.xy_res-map.xy_res);
        y = linspace(map.block(i,2)*map.xy_res-map.xy_res, map.block(i,5)*map.xy_res-map.xy_res);
        z = linspace(map.block(i,3)*map.z_res-map.z_res, map.block(i,6)*map.z_res-map.z_res);
        [X1,Y1] = meshgrid(x,y);
        [X2,Z1] = meshgrid(x,z);
        [Y2,Z2] = meshgrid(y,z);
%         for j = 0:1
         j=0;
         X =  linspace(map.block(i,1+3*j)*map.xy_res-map.xy_res, map.block(i,1+3*j)*map.xy_res);
         Y =  linspace(map.block(i,2+3*j)*map.xy_res-map.xy_res, map.block(i,2+3*j)*map.xy_res);
         Z = linspace(map.block(i,3+3*j)*map.z_res-map.z_res, map.block(i,3+3*j)*map.z_res);
            if i ~= 1 
               plot3(X1,Y1,Z,'k');hold on;
               plot3(X2,Y,Z1,'k');hold on;
               plot3(X,Y2,Z2,'k');hold on;
            end
%         end
end
plot3(path(:,1),path(:,2),path(:,3),'r','LineWidth',2,'MarkerFaceColor','b','MarkerSize',3);
grid on;
hold on;
toc

end