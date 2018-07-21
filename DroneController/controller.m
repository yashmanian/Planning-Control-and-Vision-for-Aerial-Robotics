function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

r = qd{qn}.pos; 
r_dot = qd{qn}.vel; 
r_des = qd{qn}.pos_des; 
r_des_dot = qd{qn}.vel_des; 
r_des_ddot = qd{qn}.acc_des;

% Find errors ep, ev
ep = r_des-r;
ev = r_des_dot -r_dot;

% Solve for rddot des
kp = [14; 7; 30];
kd = [16; 10; 5]; 

rddot_des = kp.*ep + kd.*ev + r_des_ddot;

% Get phi_des, theta_des, u1_des
u1 = (params.grav + rddot_des(3))*params.mass;
phi_d = (1/params.grav)*(rddot_des(1)*sin(qd{qn}.yaw_des)-rddot_des(2)*cos(qd{qn}.yaw_des));
theta_d = (1/params.grav)*(rddot_des(1)*cos(qd{qn}.yaw_des)-rddot_des(2)*sin(qd{qn}.yaw_des));

% Get desired u2 u3 u4
kp_ang = 0.1; 
kd_ang = 0.02; 

psi_d = qd{qn}.yaw_des;

u2des = kp_ang*(phi_d-qd{qn}.euler(1))+kd_ang*(0-qd{qn}.omega(1));

u3des = kp_ang*(theta_d-qd{qn}.euler(2))+kd_ang*(0-qd{qn}.omega(2));

u4des = kp_ang*(psi_d-qd{qn}.euler(3));

% Desired roll, pitch and yaw
phi_des = phi_d;
theta_des = theta_d;
psi_des = 0;


% Thurst
F    = u1; 
% Moment
M    = [u2des; u3des; 0]; % You should fill this in

% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
