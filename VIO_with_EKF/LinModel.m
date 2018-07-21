function [model_func, At_func, Ut_func, state_func] = LinModel()
%% Variables
syms x y z x_dot y_dot z_dot phi theta psii dt
am = sym('am', [3,1]);
wm = sym('wm', [3,1]);
ng = sym('ng', [3,1]);
na = sym('na', [3,1]);
bg = sym('bg', [3,1]);
ba = sym('ba', [3,1]);
nbg = sym('nbg', [3,1]);
nba = sym('nba', [3,1]);
p = [x;y;z];
p_dot = [x_dot; y_dot; z_dot];
q = [phi;theta;psii];
g = [0;0;-9.81];
ni = sym('ni', [3,1]);
n = [ng;ng;na;nbg;nba];
%% Transforms
G = [cos(theta) 0 -cos(phi)*sin(theta); 0 1 sin(phi); sin(theta) 0 cos(phi)*cos(theta)];
rot = [cos(psii)*cos(theta) - sin(phi)*sin(psii)*sin(theta), cos(theta)*sin(psii) + cos(psii)*sin(phi)*sin(theta), -cos(phi)*sin(theta);-cos(phi)*sin(psii), cos(phi)*cos(psii), sin(phi);cos(psii)*sin(theta) + cos(theta)*sin(phi)*sin(psii), sin(psii)*sin(theta) - cos(psii)*cos(theta)*sin(phi), cos(phi)*cos(theta)];

%% System equations
X = [p;q;p_dot;bg;ba];
model = [p_dot;G\(wm-bg-ng); g + rot*(am-ba-na);nbg;nba];

%% Jacobians
At = jacobian(model,X);
Ut = jacobian(model,n);

Ft = eye(15) + At*dt;
Vt = Ut*dt;

%% Functions
state_func = matlabFunction(X);
model_func = matlabFunction(model);
At_func = matlabFunction(At);
Ut_func = matlabFunction(Ut);

end
