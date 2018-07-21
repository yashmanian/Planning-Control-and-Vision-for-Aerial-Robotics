function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% CMSC 828T Proj 1 Phase 1

%% START YOUR CODE HERE %%
map_x = size(map{1},1);
map_y = size(map{1},2);
map_z = size(map{1},3);

figure
plot3(path(:,1),path(:,2),path(:,3),'r')
axis([1 map_x 1 map_y 1 map_z])
PATCH_3Darray(map{1})
%% END YOUR CODE HERE %%

end