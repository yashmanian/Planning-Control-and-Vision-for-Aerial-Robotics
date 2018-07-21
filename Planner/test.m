clear all
xy_res = 0.5;
z_res = 0.5;
margin = 0.25;
start = [1 1 1];
goal = [7 18 4];
T = 10;
dt = 0.001;

map = load_map('map0.txt', xy_res, z_res, margin);
%%
[path, num_expanded] = dijkstra(map, start, goal, 1);

%%
plot_path(map, path)
