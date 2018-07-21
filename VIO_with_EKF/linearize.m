%clear all
function [model_func, Ft_func, Vt_func, state_func] = linearize(IMU_bias)
dt = sym('dt'); % Sample time
X = sym('X', [15,1]); % State

% Measurement and noise
Z = sym('Z', [6,1]);
v = sym('v', [9,1]);

% Angles
phi = X(4);
theta = X(5);
psi = X(6);
rot1 = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta);-cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi);cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(theta)];

meas = Z + v(1:6);
phi2 = meas(4);
theta2 = meas(5);
psi2 = meas(6);
rot2 = [cos(psi2)*cos(theta2) - sin(phi2)*sin(psi2)*sin(theta2), cos(theta2)*sin(psi2) + cos(psi2)*sin(phi2)*sin(theta2), -cos(phi2)*sin(theta2);-cos(phi2)*sin(psi2), cos(phi2)*cos(psi2), sin(phi2);cos(psi2)*sin(theta2) + cos(theta2)*sin(phi2)*sin(psi2), sin(psi2)*sin(theta2) - cos(psi2)*cos(theta2)*sin(phi2), cos(phi2)*cos(theta2)];

rot3 = rot1*rot2;

phi3 = asin(rot3(2,3));
theta3 = atan2(-rot3(1,3)/cos(phi3),rot3(3,3)/cos(phi3));
psi3 = atan2(-rot3(2,1)/cos(phi3),rot3(2,2)/cos(phi3));
rpy = [phi3; theta3; psi3];

% Process model
f = [X(1:3) + X(7:9)*dt + (((rot1 * (Z(1:3) - X(10:12) + v(1:3))) - IMU_bias)* dt^2 / 2);rpy;X(7:9) + (((rot1 * (Z(1:3) - X(10:12) + v(1:3))) - IMU_bias)* dt);X(10:12) + v(7:9)*dt;X(13:15)];
f = simplify(f);

% Linearize with the jacobians
Ft = jacobian(f, X);
Vt = jacobian(f, v);

%% Functions
state_func = matlabFunction(X);
model_func = matlabFunction(f);
Ft_func = matlabFunction(Ft);
Vt_func = matlabFunction(Vt);

end