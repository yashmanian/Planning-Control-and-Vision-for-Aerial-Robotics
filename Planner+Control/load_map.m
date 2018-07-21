function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  an obstacle.

% CMSC 828T Proj 1 Phase 1

% Output structure: 
    % Map is a cell array containing the following:
    %   --> map{1} contains a 3D logical occupancy grid
    %   --> map{2}, Map{3} and Map{4} store the discretized axes values
    %       corresponding to the X, Y and Z axes respectively
    %   --> map{5} and map{6} store the xy_res and z_res respectively

% filename = ('map0.txt');
% xy_res = 0.1;
% z_res = 1;
% margin = 0;

% Open file, read data, close file. Comments in file marked by #
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);

%% START YOUR CODE HERE %%
name = fileDat{1};
x_init = fileDat{2};
y_init = fileDat{3};
z_init = fileDat{4};
x_end = fileDat{5};
y_end = fileDat{6};
z_end = fileDat{7};
R = fileDat{8};
G = fileDat{9};
B = fileDat{10};
%%
% Boundary constraints
[p,qbound] = ismember(name,'boundary');
qbound = find(qbound);
boundary = [x_init(qbound) y_init(qbound) z_init(qbound); x_end(qbound) y_end(qbound) z_end(qbound)];
[p,qblock] = ismember(name,'block');
qblock = find(qblock);
block_lim = [x_init(qblock) y_init(qblock) z_init(qblock) x_end(qblock) y_end(qblock) z_end(qblock) R(qblock) G(qblock) B(qblock)];

%%
x = x_init(qbound):xy_res:x_end(qbound);
y = y_init(qbound):xy_res:y_end(qbound);
z = z_init(qbound):z_res:z_end(qbound);
x_nodes = [x_init(qbound):xy_res:(x_end(qbound)-xy_res)] + (xy_res/2);
y_nodes = [y_init(qbound):xy_res:(y_end(qbound)-xy_res)] + (xy_res/2);
z_nodes = [z_init(qbound):z_res:(z_end(qbound)-z_res)] + (z_res/2);
x_dim = size(x,2);
y_dim = size(y,2);
z_dim = size(z,2);

%%
% All workspace points
workspace = [];
n = 1;

for i = 1:size(x_nodes,2) 
    for j = 1:size(y_nodes,2) 
        temp = [repmat(x_nodes(i),size(z_nodes,2),1), repmat(y_nodes(j),size(z_nodes,2),1), z_nodes'];
        workspace = [workspace;temp];
    end
end

%%
occupancyGrid = zeros(size(x_nodes,2), size(y_nodes,2), size(z_nodes,2));

for a = 1:size(block_lim,1)
    curr_block_init = block_lim(a,1:3);
    curr_block_end = block_lim(a,4:6);
    
    margin_a1 = margin;
    margin_a2 = margin;
    margin_a3 = margin;
    margin_b1 = margin;
    margin_b2 = margin;
    margin_b3 = margin;
    
    if curr_block_init(1) == 1
        margin_a1 = 0;
    end
    
    if curr_block_init(2) == 1
        margin_a2 = 0;
    end
    
    if curr_block_init(3) == 1
        margin_a3 = 0;
    end
    
    if curr_block_end(1) == x_dim
        margin_b1 = 0;
    end
    
    if curr_block_end(2) == y_dim
        margin_b2 = 0;
    end
    
    if curr_block_end(3) == z_dim
        margin_b3 = 0;
    end
    
    x_start = curr_block_init(1) - margin_a1;
    x_stop = curr_block_end(1) + margin_b1;
    
    y_start = curr_block_init(2) - margin_a2;
    y_stop = curr_block_end(2) + margin_b2;
    
    z_start = curr_block_init(3) - margin_a3;
    z_stop = curr_block_end(3) + margin_b3;
    
    
    
    x_block_idx = x_nodes <= x_stop & x_nodes >= x_start;
    x_block_idx = find(x_block_idx);
    x_block = x_nodes(x_block_idx);
    
    y_block_idx = y_nodes <= y_stop & y_nodes >= y_start;
    y_block_idx = find(y_block_idx);
    y_block = y_nodes(y_block_idx);
    
    z_block_idx = z_nodes <= z_stop & z_nodes >= z_start;
    z_block_idx = find(z_block_idx);
    z_block = z_nodes(z_block_idx);
    
    x_grid = ((x_block - x_nodes(1))/xy_res) + 1;
    y_grid = ((y_block - y_nodes(1))/xy_res) + 1;
    z_grid = ((z_block - z_nodes(1))/z_res) + 1;
    
    occupancyGrid(uint16(x_grid),uint16(y_grid),uint16(z_grid)) = 1;
end
x_i = uint16(((workspace(:,1) - x_nodes(1))/xy_res) + 1);
y_i = uint16(((workspace(:,2) - y_nodes(1))/xy_res) + 1);
z_i = uint16(((workspace(:,3) - z_nodes(1))/z_res) + 1);
gridspace = [x_i y_i z_i];

%% Output
map = cell(1,7);
map{1} = occupancyGrid;
map{2} = x;
map{3} = y;
map{4} = z;
map{5} = xy_res;
map{6} = z_res;
map{7} = workspace;
map{8} = gridspace;
%PATCH_3Darray(map{1})
%% END YOUR CODE HERE %%
end
