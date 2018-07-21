% Add additional initialization if you want to.
% You can use this space to pre-compute the trajectory to avoid
% repeatedly computing the same trajectory in every call of the
% "trajectory_generator" function

global tspan coeffs_x coeffs_y coeffs_z
tspan = [0];
t0 = 0;
ta = 0;
len = size(path{1},1);
dist = [];
d_pos = [];
new_path = double(path{1});
T = 15;

for j=1:(len-1)
    delta_pos = new_path(j+1,:) - new_path(j,:);
    euclid_pos = sqrt(delta_pos(1)^2 + delta_pos(2)^2 + delta_pos(3)^2);
    dist = [dist;euclid_pos];
    d_pos = [d_pos;delta_pos];
end

delta_t = T*(dist/sum(dist));
delta_v = d_pos./delta_t;
delta_v = [[0 0 0];delta_v];
delta_v(end,:) = [0 0 0];
delta_t = [0;delta_t];
t_rec = [];
coeffs_x = [];
coeffs_y = [];
coeffs_z = [];
tspan = [0];
%%
for i = 1:(len-1)
    %%
    tf = t0 + delta_t(i+1);
    v0 = delta_v(i,:);
    vf = delta_v(i+1,:);
    p0 = new_path(i,:);
    p1 = new_path(i+1,:);
%%
    M = [1 t0 t0^2 t0^3 t0^4 t0^5; 
    0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
    0 0 2 6*t0 12*t0^2 20*t0^3; 
    1 tf tf^2 tf^3 tf^4 tf^5; 
    0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
    0 0 2 6*tf 12*tf^2 20*tf^3];

    %B = [p0;v0;[0 0 0];p1;vf;[0 0 0]];
    B = [p0;[0 0 0];[0 0 0];p1;[0 0 0];[0 0 0]];
    A = inv(M)*B;
  %%
    coeffs_x = [coeffs_x;A(:,1)'];
    coeffs_y = [coeffs_y;A(:,2)'];
    coeffs_z = [coeffs_z;A(:,3)'];
    
    tspan = [tspan;tf];
    t_rec = [t_rec;[t0 tf]];
    t0 = tf;
end

% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], [], map, path);
