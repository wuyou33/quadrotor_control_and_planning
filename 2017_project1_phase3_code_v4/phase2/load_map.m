function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle. xy_res, z_res are resolution.

fileID = fopen(filename); 
data = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);

% assume boundary is always 1st row, the rest are blocks
n=size(data{1},1); 
% check the row of boundary
[~,m]=ismember('boundary', data{1,1});

% boundary 
boundary_xmin= data{2}(m)/xy_res;
boundary_ymin= data{3}(m)/xy_res;
boundary_zmin= data{4}(m)/z_res;
boundary_xmax= data{5}(m)/xy_res;
boundary_ymax= data{6}(m)/xy_res;
boundary_zmax= data{7}(m)/z_res;

boundary_sz(1)= ceil(boundary_xmax-boundary_xmin)+1;
boundary_sz(2)= ceil(boundary_ymax-boundary_ymin)+1;
boundary_sz(3)= ceil(boundary_zmax-boundary_zmin)+1;
table = zeros(boundary_sz(1),boundary_sz(2),boundary_sz(3));

margin_xy = margin/xy_res;
margin_z = margin/z_res;

if n >= 2 % for emptymap
    
for j = 1:n
    if j ~= m
    % enlarge blocks, problem here
    block_xmin=  ceil(data{2}(j)/xy_res-boundary_xmin-margin_xy+1);
    block_ymin=  ceil(data{3}(j)/xy_res-boundary_ymin-margin_xy+1);
    block_zmin=  ceil(data{4}(j)/z_res-boundary_zmin-margin_z+1);
    block_xmax=  ceil(data{5}(j)/xy_res-boundary_xmin+margin_xy+1);
    block_ymax=  ceil(data{6}(j)/xy_res-boundary_ymin+margin_xy+1);
    block_zmax=  ceil(data{7}(j)/z_res-boundary_zmin+margin_z+1);
    if block_xmin<1
        block_xmin=1;
    end
    if block_ymin<1
        block_ymin=1;
    end
    if block_zmin<1
        block_zmin=1;
    end
    if block_xmax >(boundary_sz(1))
        block_xmax = (boundary_sz(1));
    end 
    if block_ymax >(boundary_sz(2))
        block_ymax = (boundary_sz(2));
    end 
    if block_zmax >(boundary_sz(3))
        block_zmax = (boundary_sz(3));
    end 
    map.block(j,:) = [block_xmin block_ymin block_zmin block_xmax block_ymax block_zmax];
    table(block_xmin:block_xmax,block_ymin:block_ymax,block_zmin:block_zmax)=1;
    end
end
end

map.boundary_sz = boundary_sz;
map.boundary_min = [ceil(boundary_xmin) ceil(boundary_ymin) ceil(boundary_zmin)]; % might be problem here
map.block(1,:) =[ceil(boundary_xmin) ceil(boundary_ymin) ceil(boundary_zmin) ceil(boundary_xmax) ceil(boundary_ymax) ceil(boundary_zmax)];
map.boundary_z= data{7}(1)- data{4}(1);
map.z_res = z_res;
map.xy_res = xy_res;
map.table = table;

end
