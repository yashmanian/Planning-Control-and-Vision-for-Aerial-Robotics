function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond
% DIAMOND IS UNBREAKABLE
% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
t0 = 0;
tf_max = 4;
p2 = 1;


if t< (tf_max/4)
    B = [0 0 0; 0 0 0; 0 0 0; 0 p2 p2; 0 0 0; 0 0 0];
    t0 = 0;
    tf = tf_max/4;
else
    if t < (tf_max/2)
        B = [0 p2 p2; 0 0 0; 0 0 0; 0 0 2*p2; 0 0 0; 0 0 0];
        t0 = (tf_max/4);
        tf = (tf_max/2);
    else
        if t < 3*(tf_max/4)
            B = [0 0 2*p2; 0 0 0; 0 0 0; 0 -p2 p2; 0 0 0; 0 0 0];
            t0 = (tf_max/2);
            tf = 3*(tf_max/4);
        else
            B = [0 -p2 p2; 0 0 0; 0 0 0; 1 0 0; 0 0 0; 0 0 0];
            t0 = 3*(tf_max/4);
            tf = tf_max;
        end
    end
end

M = [1 t0 t0^2 t0^3 t0^4 t0^5; 
    0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
    0 0 2 6*t0 12*t0^2 20*t0^3; 
    1 tf tf^2 tf^3 tf^4 tf^5; 
    0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
    0 0 2 6*tf 12*tf^2 20*tf^3];

A = inv(M)*B;

a = A(1,1);
b = A(2,1);
c = A(3,1);
d = A(4,1);
e = A(5,1);
f = A(6,1);

x_t = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
x_t_dot = 5*f*t^4 + 4*e*t^3 + 3*d*t^2 + 2*c*t + b;
x_t_ddot = 2*c + 6*d*t + 12*e*(t^2) + 20*f*(t^3);


a = A(1,2);
b = A(2,2);
c = A(3,2);
d = A(4,2);
e = A(5,2);
f = A(6,2);

y_t = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
y_t_dot = 5*f*t^4 + 4*e*t^3 + 3*d*t^2 + 2*c*t + b;
y_t_ddot = 2*c + 6*d*t + 12*e*(t^2) + 20*f*(t^3);


a = A(1,3);
b = A(2,3);
c = A(3,3);
d = A(4,3);
e = A(5,3);
f = A(6,3);

z_t = a + b*t + c*t^2 + d*t^3 + e*t^4 + f*t^5;
z_t_dot = 5*f*t^4 + 4*e*t^3 + 3*d*t^2 + 2*c*t + b;
z_t_ddot = 2*c + 6*d*t + 12*e*(t^2) + 20*f*(t^3);

x = x_t;
y = y_t;
z = z_t;

x_dot = x_t_dot;
y_dot = y_t_dot;
z_dot = z_t_dot;

x_ddot = x_t_ddot;
y_ddot = y_t_ddot;
z_ddot = z_t_ddot;

pos = [x y z];
vel = [x_dot y_dot z_dot];
acc = [x_ddot y_ddot z_ddot];

if t >= tf
    pos = [1 0 0];
    vel = [0 0 0];
    acc = [0 0 0];
end

yaw = 0;
yawdot = 0;
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
