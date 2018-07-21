close all;
clear all;
clc;

%% Plan path
disp('Planning ...');
map = load_map('maps/map3.txt', 0.5, 0.5, 0.25);
start = {[1  3 5]};
stop  = {[19  3 5]};
nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
