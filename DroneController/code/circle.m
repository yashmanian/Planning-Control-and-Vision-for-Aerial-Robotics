function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
r = 5;
t0 = 0;
tf = 20;
z_max = 2.5;

if t <= tf
    M = [1 t0 t0^2 t0^3 t0^4 t0^5; 
        0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
        0 0 2 6*t0 12*t0^2 20*t0^3; 
        1 tf tf^2 tf^3 tf^4 tf^5; 
        0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
        0 0 2 6*tf 12*tf^2 20*tf^3];
    B = [0 0; 0 0; 0 0; 2*pi z_max; 0 0; 0 0];
    A = inv(M)*B;

    a1 = A(1,1);
    b1 = A(2,1);
    c1 = A(3,1);
    d1 = A(4,1);
    e1 = A(5,1);
    f1 = A(6,1);

    a2 = A(1,2);
    b2 = A(2,2);
    c2 = A(3,2);
    d2 = A(4,2);
    e2 = A(5,2);
    f2 = A(6,2);

    xy_t = a1 + b1*t + c1*t^2 + d1*t^3 + e1*t^4 + f1*t^5;
    xy_t_dot = 5*f1*t^4 + 4*e1*t^3 + 3*d1*t^2 + 2*c1*t + b1;
    xy_t_ddot = 20*f1*2^3 + 12*e1*t^2 + 6*d1*t + 2*c1;

    z_t = a2 + b2*t + c2*t^2 + d2*t^3 + e2*t^4 + f2*t^5;
    z_t_dot = 5*f2*t^4 + 4*e2*t^3 + 3*d2*t^2 + 2*c2*t + b2;
    z_t_ddot = 20*f2*2^3 + 12*e2*t^2 + 6*d2*t + 2*c2;

    x = r*cos(xy_t);
    y = r*sin(xy_t);
    z = z_t;

    x_dot = -r*sin(xy_t)*xy_t_dot;
    y_dot = r*cos(xy_t)*xy_t_dot;
    z_dot = z_t_dot;

    x_ddot = -r*xy_t_ddot*sin(xy_t) - r*(xy_t_dot^2)*cos(xy_t);
    y_ddot = r*xy_t_ddot*cos(xy_t) - r*(xy_t_dot^2)*sin(xy_t);
    z_ddot = z_t_ddot;
else
    x = 5;
    y = 0;
    z = 2.5;
    
    x_dot = 0;
    y_dot = 0;
    z_dot = 0;

    x_ddot = 0;
    y_ddot = 0;
    z_ddot = 0;
end

pos = [x y z];
vel = [x_dot y_dot z_dot];
acc = [x_ddot y_ddot z_ddot];
yaw = 0;
yawdot = 0;
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
