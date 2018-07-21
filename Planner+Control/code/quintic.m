function [pos, vel, acc] = quintic(t0, tf, v0, vf, p0, p1, t)
    M = [1 t0 t0^2 t0^3 t0^4 t0^5; 
    0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
    0 0 2 6*t0 12*t0^2 20*t0^3; 
    1 tf tf^2 tf^3 tf^4 tf^5; 
    0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
    0 0 2 6*tf 12*tf^2 20*tf^3];
    
    B = [p0;v0;[0 0 0];p1;vf;[0 0 0]];
    %B = [p0;[0 0 0];[0 0 0];p1;[0 0 0];[0 0 0]];
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
    
    pos = [x_t y_t z_t];
    vel = [x_t_dot y_t_dot z_t_dot];
    acc = [x_t_ddot y_t_ddot z_t_ddot];
end
    