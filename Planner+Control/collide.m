function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% CMSC 828T Proj 1 Phase 1

% If the map is empty, output empty vector else, compute
C = [];
if isempty(map)
    C = [];
else
    %% START YOUR CODE HERE %%
    occupancyGrid = map{1};
    x = map{2};
    y = map{3};
    z = map{4};
    xy_res = map{5};
    z_res = map{6};
    x_nodes = [x(1):xy_res:(x(end)-xy_res)] + (xy_res/2);
    y_nodes = [y(1):xy_res:(y(end)-xy_res)] + (xy_res/2);
    z_nodes = [z(1):z_res:(z(end)-z_res)] + (z_res/2);
    map_x = size(map{1},1);
    map_y = size(map{1},2);
    map_z = size(map{1},3);

    points_x = uint16(((points(:,1) - x_nodes(1))/xy_res) + 1);
    points_y = uint16(((points(:,2) - y_nodes(1))/xy_res) + 1);
    points_z = uint16(((points(:,3) - z_nodes(1))/z_res) + 1);
    points = [points_x points_y points_z];
    
    for i = 1:size(points,1)
        if occupancyGrid(points(i,1),points(i,2), points(i,3)) == 0
            C = [C; 0];
        else
            C = [C;1];
        end
    end
    %% END YOUR CODE HERE %%
end
end
