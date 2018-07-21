function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% CMSC 828T Proj 1 Phase 1

if nargin < 4
    astar = false;
end

% xy_res = 0.5;
% z_res = 0.5;
% margin = 0.1;
% start = [1 1 1];
% goal = [7 18 4];
% astar = 1;
% map = load_map('map0.txt', xy_res, z_res, margin);

%% START YOUR CODE HERE %%
start = start';
goal = goal';

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

current_x = 1;
current_y = 2;
current_z = 3;
parent_x = 4;
parent_y = 5;
parent_z = 6;
visited = 7;

xy_res = map{5};
z_res = map{6};
final_ws = map{7};
node_list = map{8};
start_cost = 9999999;
num_expanded = 0;

% Convert coordinate to logical point
start_x = uint16(((start(1) - x_nodes(1))/xy_res) + 1);
start_y = uint16(((start(2) - y_nodes(1))/xy_res) + 1);
start_z = uint16(((start(3) - z_nodes(1))/z_res) + 1);

goal_x = uint16(((goal(1) - x_nodes(1))/xy_res) + 1);
goal_y = uint16(((goal(2) - y_nodes(1))/xy_res) + 1);
goal_z = uint16(((goal(3) - z_nodes(1))/z_res) + 1);

% Construct Data structures
start_node = [start_x start_y start_z 0 0 0];
goal_node = [goal_x goal_y goal_z 0 0 0];

% Remove start node
nodes = [node_list(:,1) node_list(:,2) node_list(:,3) repmat([0 0 0],size(node_list,1),1)];
I = find(all(bsxfun(@eq, nodes(:,1:3),start_node(1,1:3)),2));
nodes(I,:) = [];
final_ws(I,:) = [];
%%
% Collision check
C = collide(map,final_ws(:,1:3));
I = find(C<1);
unvisited=nodes(I,:);

priority_queue = [start_node];
priority_cost = [0];
visited_nodes = [];
visited_costs = [];
PQ_size1 = 0;
PQ_size2 = 0;
val = [];


%% 
while ~isempty(unvisited)
%for a = 1:27
    [u,v] = min(priority_cost);
    curr_node = priority_queue(v,:);
    curr_dist = priority_cost(v);
    priority_queue(v,:) = [];
    priority_cost(v) = [];
   

%% Get next possible nodes
    poss_nodes = [];
    next_x = [curr_node(1)-1 curr_node(1) curr_node(1)+1];
    next_y = [curr_node(2)-1 curr_node(2) curr_node(2)+1];
    next_z = [curr_node(3)-1 curr_node(3) curr_node(3)+1];
    curr_node_1 = double(curr_node);
    goal_node_1 = double(goal_node);
    
    poss_nodes = [curr_node(1)-1 curr_node(2) curr_node(3);curr_node(1)+1 curr_node(2) curr_node(3);curr_node(1) curr_node(2)-1 curr_node(3);curr_node(1) curr_node(2)+1 curr_node(3);curr_node(1) curr_node(2) curr_node(3)-1;curr_node(1) curr_node(2) curr_node(3)+1];
    next_nodes = [];
 %% Find indices of nodes
    for a = 1:size(poss_nodes,1)
        c = find(all(bsxfun(@eq, unvisited(:,1:3),poss_nodes(a,1:3)),2));
        next_nodes = [next_nodes;unvisited(c,:)];
    end
    node_val = double(next_nodes(:,1:3));
    
%% Get current node costs
    node_dist = [];
    for a = 1:size(next_nodes,1)
        p_idx = find(all(bsxfun(@eq, priority_queue(:,1:3),next_nodes(a,1:3)),2));
        if ~isempty(p_idx)
            node_dist(a,1) = priority_cost(p_idx);
        else
            node_dist(a,1) = start_cost;
        end
    end
%% Compute costs
    dist2node = sqrt((node_val(:,1)-curr_node_1(current_x)).^2 +  (node_val(:,2)-curr_node_1(current_y)).^2 +(node_val(:,3)-curr_node_1(current_z)).^2);
    heuristic = sqrt((node_val(:,1)-goal_node_1(current_x)).^2 +  (node_val(:,2)-goal_node_1(current_y)).^2 +(node_val(:,3)-goal_node_1(current_z)).^2);
    
    if astar == 1
        total_dist = curr_dist + heuristic + dist2node;
    else
        total_dist = curr_dist + dist2node;
    end

%% Change parents based on cost
    for m = 1:size(next_nodes,1)
        f = find(all(bsxfun(@eq, priority_queue(:,1:3),next_nodes(m,1:3)),2));
        if ~isempty(f) == 1
            next_nodes(m,4:6) = priority_queue(f,4:6);
        else
            continue
        end
    end
%% Cost update and parent update
    for i = 1:size(node_dist,1)
        if total_dist(i) < node_dist(i)
            node_dist(i) = total_dist(i);
            next_nodes(i,4:6) = curr_node(1:3);
        end
    end
%% Get rid of existing current node list in priority queue
    for j = 1:size(next_nodes,1)
        f = find(all(bsxfun(@eq, priority_queue(:,1:3),next_nodes(j,1:3)),2));
        if ~isempty(f)
            priority_queue(f,:) = [];
            priority_cost(f) = [];
        end
    end
    priority_queue = [priority_queue;next_nodes];
    priority_cost = [priority_cost;node_dist];
   
%% Sort priority queue and pop unvisited queue
    [priority_cost,idx] = sort(priority_cost);
    priority_queue = priority_queue(idx,:);
    visited_nodes = [visited_nodes;curr_node];
    visited_costs = [visited_costs;curr_dist];
    
    f = find(all(bsxfun(@eq, unvisited(:,1:3),curr_node(:,1:3)),2));
    unvisited(f,:) = [];
% Break condition
    if isequal(curr_node(1:3),goal_node(1:3)) & astar == 1
        num_expanded = size(visited_nodes,1);
        break
    end
end

%% Return path
q = find(all(bsxfun(@eq, visited_nodes(:,1:3),goal_node(:,1:3)),2));
node = visited_nodes(q,:);
end_parent = [0 0 0];
node_parent = node(4:6);
path = [node(1:3)];

while ~isequal(end_parent,node_parent)
    path = [path;node_parent];
    node = node_parent;
    q = find(all(bsxfun(@eq, visited_nodes(:,1:3),node(1,1:3)),2));
    node_parent = visited_nodes(q,4:6);
end

path = smooth_path(map,path);
path = flipud(path);
%% END YOUR CODE HERE %%
%plot_path(map, path)
end