clear all

% load('DataStraightLine.mat');
load('DataMountain.mat');
% load('DataSlowCircle.mat');
% load('DataFastCircle.mat');
% load('DataMapping.mat');
% load('DataSquare.mat');

%IMU inputs have to be transformed to camera frame previously
% IMU_bias = [ 0.4109; 0.4024; 9.6343];
IMU_bias = [mean(IMU(:,5)); mean(IMU(:,6)); mean(IMU(:,7)) + 9.81];

dt = sym('dt'); % Sampling time

% State
X = sym('X', [15, 1]);

% IMU measurements (control input) and error
Z = sym('Z', [6, 1]);
v = sym('v', [9, 1]);

% Angles 
phi = X(4); theta = X(5); psi = X(6);
rot1 = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta);
       -cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi);
       cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(theta)];

meas = (Z + v(1:6)) * dt;
phi2 = meas(4); theta2 = meas(5); psi2 = meas(6);
rot2 = [cos(psi2)*cos(theta2) - sin(phi2)*sin(psi2)*sin(theta2), cos(theta2)*sin(psi2) + cos(psi2)*sin(phi2)*sin(theta2), -cos(phi2)*sin(theta2);
       -cos(phi2)*sin(psi2), cos(phi2)*cos(psi2), sin(phi2);
       cos(psi2)*sin(theta2) + cos(theta2)*sin(phi2)*sin(psi2), sin(psi2)*sin(theta2) - cos(psi2)*cos(theta2)*sin(phi2), cos(phi2)*cos(theta2)];

rot3 = rot1 * rot2;
phi3 = asin(rot3(2,3));
theta3 = atan(-rot3(1,3)/cos(phi3),rot3(3,3)/cos(phi3));
psi3 = atan(-rot3(2,1)/cos(phi3),rot3(2,2)/cos(phi3));
rpy = [phi3; theta3; psi3];

   
% Process Equation
g = [X(1:3) + X(7:9)*dt + (((rot1 * (Z(1:3) - X(10:12) + v(1:3))) - IMU_bias)* dt^2 / 2);
     rpy;
     X(7:9) + (((rot1 * (Z(1:3) - X(10:12) + v(1:3))) - IMU_bias)* dt);
     X(10:12) + v(7:9)*dt;
     X(13:15)];  
g = simplify(g);

% Linearize with the jacobians
G = jacobian(g, X);
L = jacobian(g, v);

% Transform the symbolic expressions
g = matlabFunction(g);
G = matlabFunction(G);
L = matlabFunction(L);
 