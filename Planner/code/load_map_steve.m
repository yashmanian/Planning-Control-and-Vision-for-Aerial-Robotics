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



% Open file, read data, close file. Comments in file marked by #
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);

%% START YOUR CODE HERE %%
%tic
% 
  %xy_res=.1;
 % z_res =2;margin = .3;
 %filename = './sample_maps/map0.txt';


% Get columns of txt file
name = fileDat{1};
col2 = fileDat{2};
col3 = fileDat{3};
col4 = fileDat{4}; 
col5 = fileDat{5};
col6 = fileDat{6};
col7 = fileDat{7};
col8 = fileDat{8};
col9 = fileDat{9};
col10 = fileDat{10};

% Get boundary info
boundary = [col2(1); col3(1); col4(1); col5(1); col6(1); col7(1)]; % get boundary, always first line

% Get mins and max
xmin = boundary(1); xmax = boundary(4); ymin = boundary(2); ymax = boundary(5); zmin = boundary(3); zmax = boundary(6);

%Get number of obstacles and put in blocks matrix
numBlocks = size(fileDat{1},1); % find number of blocks
blocks = [];
for i = 2:numBlocks % create matrix blocks with each row block data
    newRow = [col2(i) col3(i) col4(i) col5(i) col6(i) col7(i) col8(i) col9(i) col10(i)];
    blocks = [blocks ; newRow];
end


%Turn blocks into series of points
blockPoints2 = [];
for a = 1:size(blocks,1)
    block = blocks(a,:);
    blockX = block(1):.2:block(4);
    blockY = block(2):.2:block(5);
    blockZ = block(3):.2:block(6);
    V = {blockX,blockY,blockZ}; n = 3;
    combs = cell(1,n);
    [combs{end:-1:1}] = ndgrid(V{end:-1:1});
    combs = cat(n+1, combs{:});
    combs = reshape(combs,[],n); % all x y z points in this block
    blockPoints2 = [blockPoints2 ; combs];  
end
 
% Discretized axes values
xAxis = [xmin:xy_res:xmax]';
yAxis = [ymin:xy_res:ymax]';
zAxis = [zmin:z_res:zmax]';
xNodes = [xmin:xy_res:xmax-xy_res]' + (xy_res/2);
yNodes = [ymin:xy_res:ymax-xy_res]' + (xy_res/2);
zNodes = [zmin:z_res:zmax-z_res]' + (z_res/2);
% Create All Points
%allNodes = [xNodes(:];
V = {xNodes,yNodes,zNodes}; n = 3;
combs = cell(1,n);
[combs{end:-1:1}] = ndgrid(V{end:-1:1});
combs = cat(n+1, combs{:});
allNodes = reshape(combs,[],n); % all x y z points in this block, Mx3 vector    

% Create logical grid
occGrid = zeros(size(xNodes,1), size(yNodes,1), size(zNodes,1));

% Find points in grid w/ obstacles
for a = 1:size(blockPoints2,1)
    for b = 1:size(allNodes)
        dx = blockPoints2(a,1)-allNodes(b,1); 
        dy = blockPoints2(a,2)-allNodes(b,2); 
        dz = blockPoints2(a,3)-allNodes(b,3);
        dist = (dx.*dx +dy.*dy +dz.*dz)^.5;
        if dist < margin
            xGrid = (allNodes(b,1)-xNodes(1))/xy_res + 1;
            yGrid = (allNodes(b,2)-yNodes(1))/xy_res + 1;
            zGrid = (allNodes(b,3)-zNodes(1))/z_res + 1;
            occGrid(int8(xGrid),int8(yGrid),int8(zGrid)) = 1;
        end
    end
end
%toc
map = cell(6,1);
map{1} = occGrid;
map{2} = xAxis;
map{3} = yAxis;
map{4} = zAxis;
map{5} = xy_res;
map{6} = z_res;
map{7} = allNodes;
%% END YOUR CODE HERE %%

end