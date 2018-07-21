function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;
persistent map0 path0
if nargin == 4
    map0 = map;
    path0 = double(path{1});
end
if nargin == 2 
    global tspan coeffs_x coeffs_y coeffs_z
    t_edges = tspan;
    t_edges = [t_edges;t];
    chrono_t = sortrows(t_edges);
    t_idx =  find(all(bsxfun(@eq, chrono_t, t ),2));
    %%
    if t == 0
        curr_t0 = 0;
        curr_coeff_x = coeffs_x(1,:);
        curr_coeff_y = coeffs_y(1,:);
        curr_coeff_z = coeffs_z(1,:);
    elseif t >= tspan(end)
        t_idx = size(tspan,1);
        curr_t0 = t_edges(t_idx-1,1);
        curr_coeff_x = coeffs_x(t_idx-1,:);
        curr_coeff_y = coeffs_y(t_idx-1,:);
        curr_coeff_z = coeffs_z(t_idx-1,:);
    else
        curr_t0 = t_edges(t_idx-1,1);
        curr_coeff_x = coeffs_x(t_idx-1,:);
        curr_coeff_y = coeffs_y(t_idx-1,:);
        curr_coeff_z = coeffs_z(t_idx-1,:);
    end
    %%
    %t = t - curr_t0;
    a = curr_coeff_x(1);
    b = curr_coeff_x(2);
    c = curr_coeff_x(3);
    d = curr_coeff_x(4);
    e = curr_coeff_x(5);
    f = curr_coeff_x(6);
    
    x_t = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    x_t_dot = 5*f*t^4 + 4*e*t^3 + 3*d*t^2 + 2*c*t + b;
    x_t_ddot = 2*c + 6*d*t + 12*e*(t^2) + 20*f*(t^3);


    a = curr_coeff_y(1);
    b = curr_coeff_y(2);
    c = curr_coeff_y(3);
    d = curr_coeff_y(4);
    e = curr_coeff_y(5);
    f = curr_coeff_y(6);

    y_t = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    y_t_dot = 5*f*t^4 + 4*e*t^3 + 3*d*t^2 + 2*c*t + b;
    y_t_ddot = 2*c + 6*d*t + 12*e*(t^2) + 20*f*(t^3);


    a = curr_coeff_z(1);
    b = curr_coeff_z(2);
    c = curr_coeff_z(3);
    d = curr_coeff_z(4);
    e = curr_coeff_z(5);
    f = curr_coeff_z(6);

    z_t = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
    z_t_dot = 5*f*t^4 + 4*e*t^3 + 3*d*t^2 + 2*c*t + b;
    z_t_ddot = 2*c + 6*d*t + 12*e*(t^2) + 20*f*(t^3);

    pos1 = [x_t y_t z_t];
    vel1 = [x_t_dot y_t_dot z_t_dot];
    acc1 = [x_t_ddot y_t_ddot z_t_ddot];

    pos = [x_t y_t z_t];
    vel = [x_t_dot y_t_dot z_t_dot];
    acc = [x_t_ddot y_t_ddot z_t_ddot];
    % =================== Your code ends here ===================
    yaw = 0;
    yawdot = 0;
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
end
    
end