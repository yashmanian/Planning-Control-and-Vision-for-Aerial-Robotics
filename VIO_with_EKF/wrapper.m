clear all
clc
%% Variables
ImageFolder = 'Images/MappingFrames';
data = matfile('DataMapping.mat');
data1 = matfile('PosesComputed_Mapping.mat');
params = matfile('CalibParams.mat');
LandMarks = matfile('LandMarksComputed.mat');
GT_pose = data.PoseGTSAM;
GT_vel = data.VelGTSAM;
iSAM_pose = data.PoseiSAM2;
iSAM_vel = data.VeliSAM2;
ImageList = ls(ImageFolder);
ImageList(1:2,:) = [];

GT_quat = GT_pose(:,4:7);
GT_eul = quat2eul(GT_quat);

iSAM_quat = iSAM_pose(:,4:7);
iSAM_eul = quat2eul(iSAM_quat);

TagSize = params.TagSize;
K = params.K; 
DetAll = data.DetAll;
TLeftImgs = data.TLeftImgs;
poses = data.PoseiSAM2;
velocities = data.VeliSAM2;
g = [0;0;-9.81];
qIMUToC = params.qIMUToC;
TIMUToC = params.TIMUToC;
IMU = data.IMU;

%% Linearize
[model_func, At_func, Ut_func, state_func] = LinModel();

%% Noise parameters
% Noise Covariances
Q = eye(15)*.1;
Qp = .1;
Qq = .1;
Qpdot = 100;
Qn = .1;
Q(1,1) = Qp; Q(2,2) = Qp; Q(3,3) = Qp;
Q(4,4) = Qq; Q(5,5) = Qq; Q(6,6) = Qq;
Q(7,7) = Qpdot; Q(8,8) = Qpdot; Q(9,9) = Qpdot;
Q(10,10) = Qn; Q(11,11) = Qn ; Q(12,12) = Qn;
Q(13,13) = Qn; Q(14,14) = Qn; Q(15,15) = Qn;

R = eye(15)*.01;
Rp = 15;
Rq = 15;
Rpdot = 0.1;
Rn = .1;

% Measurement model
ci = eye(3);
cz = zeros(3,3);
C = [ci cz cz cz cz; cz ci cz cz cz; cz cz ci cz cz];
W = 0.5*C;

% IMU noise
na = random('norm',0,0.01,3,1);
ng = random('norm',0,0.01,3,1);
ba = [mean(IMU(:,5)); mean(IMU(:,6)); mean(IMU(:,7)) + 9.81];
bg = [mean(IMU(:,8)); mean(IMU(:,9)); mean(IMU(:,10))];
nba = [var(IMU(:,5));var(IMU(:,6));var(IMU(:,7))];
nbg = [var(IMU(:,8));var(IMU(:,9));var(IMU(:,10))];

%% Data management
T_IMU = data.IMU(:,11); 
bad_idx = [];
good_idx = [];
acc_aligned = [];
w_aligned = [];

for i = 1:size(TLeftImgs) 
    diff = abs(TLeftImgs(i) - T_IMU);
    [minVal,c] = min(diff);
    if minVal > 0.3 ||( poses(i,1) == 0 && poses(i,2) == 0 && poses(i,3) == 0 && poses(i,4) ==0 && poses(i,5) == 0) 
        bad_idx = [bad_idx ; i]; 
        
    elseif minVal <= 0.3
        good_idx = [good_idx ; T_IMU(c) ]; 
        acc_aligned = [acc_aligned ; data.IMU(c,5:7)]; 
        w_aligned = [w_aligned ; data.IMU(c,8:10)]; 
    end
end

% Delete rows that do not align
TLeftImgs(bad_idx,:) = [];
velocities(bad_idx,:) = [];
poses(bad_idx,:) = [];

% Position and orientation
position = poses(:,1:3); 
quaternion = poses(:,4:7); 
angles = quat2eul(quaternion);

% Transformation matrix
Rot_IMU = quat2rotm(qIMUToC');
w_aligned = Rot_IMU*w_aligned' + TIMUToC;
w_aligned = w_aligned';

%% EKF
updatedState = zeros(size(position,1),15);

for i = 1:size(position,1)
    % Initial Parameters
    if i == 1 
        p = position(i,:)'; 
        pdot = velocities(i,:)'; 
        psi = angles(i,1);
        theta = angles(i,2);  
        phi = angles(i,3); 
        q = [phi;theta;psi];
        updatedState(i,:) = [p'  q'  pdot'  bg'  ba'];
        covariances{i} = zeros(15);
        continue
    end
    % Previous mean
    prev_mean = updatedState(i-1,:)';
    p = prev_mean(1:3);
    q = prev_mean(4:6);
    p_dot = prev_mean(7:9);
    
    phi = q(1);
    theta = q(2);
    psi = q(3);
    
    % Transformations
    rot = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), -cos(phi)*sin(theta);-cos(phi)*sin(psi), cos(phi)*cos(psi), sin(phi);cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi), cos(phi)*cos(theta)];
    G = [cos(theta) 0 -cos(phi)*sin(theta); 0 1 sin(phi); sin(theta) 0 cos(phi)*cos(theta)];
    
    % IMU values
    wm_prev = w_aligned(i-1,:)' + bg + ng;
    am_prev = acc_aligned(i-1,:)';
    dt = TLeftImgs(i)-TLeftImgs(i-1);
    
    % Process Model
    X = [ p ; q ; p_dot ; bg ; ba];
    X_dot = [p_dot ; G \ (wm_prev - bg - ng) ;  g + rot*(am_prev-ba-na) ; nbg ; nba ];
    
    % Jacobians
    At = At_func(am_prev(1),am_prev(2),am_prev(3),ba(1),ba(2),ba(3),bg(1),bg(2), bg(3),na(1),na(2),na(3),ng(1),ng(2),ng(3),phi,psi,theta,wm_prev(1),wm_prev(2),wm_prev(3)); 
    Ut = Ut_func(phi,psi,theta);
    
    Ft = eye(15) + At*dt;
    Vt = Ut*dt;
    
    prev_cov = covariances{i-1};
    
    % Prediction
    p_mean = X + X_dot*dt;
    p_cov = Ft*prev_cov*Ft' + Vt*Q*Vt';
    
    % Measurements
    p = position(i,:)';
    p_dot = velocities(i,:)';
    phi = angles(i,3);
    theta = angles(i,2);
    psi = angles(i,1);
    q = [phi;theta;psi];
    
    Z = [p;q;p_dot];
    
    
    % Kalman gain
    Kt = p_cov*C' * (C*p_cov*C' + W*R*W')^-1;
    
    % Update
    e_mean = p_mean + Kt*(Z-p_mean(1:9));
    e_cov = p_cov -Kt*C*p_cov;
    
    % Array
    updatedState(i,:) = e_mean';
    covariances{i} = p_cov;
end

%% Plot
figure
scatter3(updatedState(:,1),updatedState(:,2),updatedState(:,3))
hold on;
scatter3(GT_pose(:,1),GT_pose(:,2),GT_pose(:,3))
hold on;
scatter3(iSAM_pose(:,1),iSAM_pose(:,2),iSAM_pose(:,3))
legend('EKF', 'GTSAM', 'iSAM')
title('Position')

% Velocity
figure
subplot(3,1,1)
plot(1:size(updatedState,1), updatedState(:,7))
hold on;
plot(1:size(GT_vel,1), GT_vel(:,1))
hold on;
plot(1:size(iSAM_vel,1), iSAM_vel(:,1))
legend('EKF', 'GTSAM', 'iSAM')
title('X component Velocity')

subplot(3,1,2)
plot(1:size(updatedState,1), updatedState(:,8))
hold on;
plot(1:size(GT_vel,1), GT_vel(:,2))
hold on;
plot(1:size(iSAM_vel,1), iSAM_vel(:,2))
legend('EKF', 'GTSAM', 'iSAM')
title('Y component Velocity')

subplot(3,1,3)
plot(1:size(updatedState,1), updatedState(:,9))
hold on;
plot(1:size(GT_vel,1), GT_vel(:,3))
hold on;
plot(1:size(iSAM_vel,1), iSAM_vel(:,3))
legend('EKF', 'GTSAM', 'iSAM')
title('Z component Velocity')

% Angles
figure
subplot(3,1,1)
plot(1:size(updatedState,1), updatedState(:,4))
hold on;
plot(1:size(GT_vel,1), GT_eul(:,1))
hold on;
plot(1:size(iSAM_vel,1), iSAM_eul(:,1))
legend('EKF', 'GTSAM', 'iSAM')
title('X component Orientation')

subplot(3,1,2)
plot(1:size(updatedState,1), updatedState(:,5))
hold on;
plot(1:size(GT_vel,1), GT_eul(:,2))
hold on;
plot(1:size(iSAM_vel,1), iSAM_eul(:,2))
legend('EKF', 'GTSAM', 'iSAM')
title('Y component Orientation')

subplot(3,1,3)
plot(1:size(updatedState,1), updatedState(:,6))
hold on;
plot(1:size(GT_vel,1), GT_eul(:,3))
hold on;
plot(1:size(iSAM_vel,1), iSAM_eul(:,3))
legend('EKF', 'GTSAM', 'iSAM')
title('Z component Orientation')
