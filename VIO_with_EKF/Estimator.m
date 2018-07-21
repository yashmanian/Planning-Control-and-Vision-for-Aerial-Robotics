clear all
clc
%% Variables
ImageFolder = 'Images/SquareFrames';
data = matfile('DataSquare.mat');
data1 = matfile('PosesComputed_Square.mat');
params = matfile('CalibParams.mat');
LandMarks = matfile('LandMarksComputed.mat');

ImageList = ls(ImageFolder);
ImageList(1:2,:) = [];

TagSize = params.TagSize;
K = params.K; 
DetAll = data.DetAll;
TLeftImgs = data.TLeftImgs;
PoseiSAM2 = data.PoseiSAM2;
VeliSAM2 = data.VeliSAM2;
IMU = data.IMU;
LMs = LandMarks.LMcomputed;
cameraParams = cameraParameters('IntrinsicMatrix', K');
T = params.TIMUToC;
qIMUToC = params.qIMUToC;
R = quat2rotm(qIMUToC');
IMUToC = [R T; 0 0 0 1];

tIMU = IMU(:,11);
tIMG = TLeftImgs;

tIMU_i = zeros(size(tIMG,1),2);

%% System Equations
% Noise parameters
Q = repmat(0.1, 15,1);
Q = diag(Q);
R = repmat(0.01, 9,1);
R = diag(R);

%% IMU data alignment
for i = 1:size(tIMG)
    tIMG_i = tIMG(i);
    [t,idx] = min(abs(tIMG_i-tIMU));
    tIMU_i(i,:) = [idx,tIMU(idx)];
end
[u, ia, ic] = unique(tIMU_i(:,1));
T_IMG = [ia tIMG(ia,1)]; 
Pose_aligned = PoseiSAM2(ia,:);
Vel_aligned = VeliSAM2(ia,:);
T_IMU = [u tIMU(u)];
IMU_aligned = IMU(u,:);

na = random('norm',0,0.01,3,1);
ng = random('norm',0,0.01,3,1);
ba = [mean(IMU_aligned(:,5)); mean(IMU_aligned(:,6)); mean(IMU_aligned(:,7)) + 9.81];
bg = [mean(IMU_aligned(:,8)); mean(IMU_aligned(:,9)); mean(IMU_aligned(:,10))];
nba = [var(IMU_aligned(:,5));var(IMU_aligned(:,6));var(IMU_aligned(:,7))];
nbg = [var(IMU_aligned(:,8));var(IMU_aligned(:,9));var(IMU_aligned(:,10))];

Estimates = zeros(size(T_IMG,1),9);

prev_mean = zeros(15,1);
prev_cov = zeros(15,15);
prev_time = T_IMG(1,2) -0.0667;
prev_Zt = zeros(9,1);

%% Linearize
[model_func, Ft_func, Vt_func, state_func] = linearize2();

Ct = eye(9);
Wt = 0.5*Ct;

Estimates = zeros(size(T_IMG,1), 9);
%% Computation
for i = 1:size(T_IMG,1)
    % Transform IMU data
    IMU_t = [IMU_aligned(i,5);IMU_aligned(i,6);IMU_aligned(i,7);1];
    IMU_w = quat2eul(IMU_aligned(i,8:11))';
    IMU_w = [IMU_w;1];
    IMU_tc = IMUToC*IMU_t;
    IMU_wc = IMUToC*IMU_w;
    
    % Quaternion to Euler 
    if Pose_aligned(i,4) == 0
        w = 1;
    else
        w = Pose_aligned(i,4);
    end
    pose_eul = quat2eul([w, Pose_aligned(i,5), Pose_aligned(i,6), Pose_aligned(i,7)])';
    Zt = [Pose_aligned(i,1);Pose_aligned(i,2);Pose_aligned(i,3);pose_eul(1);pose_eul(2);pose_eul(3);Vel_aligned(i,1);Vel_aligned(i,2);Vel_aligned(i,3);bg;ba];
    
%% Process loop
    if i == 1
        prev_mean = Zt;
        prev_cov = zeros(15,15);
        prev_time = T_IMG(i,2);
        prev_am = IMU_tc;
        prev_wm = IMU_wc;
    else
        % measured state
        am = IMU_tc;
        wm = IMU_wc;
        Curr_time = T_IMG(i,2);
        dt = Curr_time - prev_time;
        p = [prev_mean(1);prev_mean(2);prev_mean(3)];
        q = [prev_mean(4);prev_mean(5);prev_mean(6)];
        p_dot = [prev_mean(7);prev_mean(8);prev_mean(9)];
        
        % Process models
        f = model_func(prev_am(1),prev_am(2),prev_am(3), ba(1), ba(2), ba(3), bg(1), bg(2), bg(3),na(1), na(2), na(3),nba(1), nba(2), nba(3), nbg(1), nbg(2), nbg(3), ng(1), ng(2), ng(3),q(1), q(3), q(2), prev_wm(1), prev_wm(2), prev_wm(3), p_dot(1), p_dot(2), p_dot(3));
        Ft = Ft_func(prev_am(1),prev_am(2),prev_am(3), ba(1), ba(2), ba(3), bg(1), bg(2), bg(3), dt, na(1), na(2), na(3), ng(1), ng(2), ng(3), q(1), q(3), q(2), prev_wm(1), prev_wm(2), prev_wm(3));
        Vt = Vt_func(dt, q(1), q(3), q(2));
        Vt(1,1) = -0.0667;
        Vt(2,2) = -0.0667;
        Vt(3,3) = -0.0667;
        % Prediction
        p_mean = prev_mean + f*dt;
        p_cov =  Ft*prev_cov*Ft' + Vt*Q*Vt';
        
        % Update
        Z = Zt(1:9);
        h = p_mean(1:9);
        
        % Kalman gain
        Kt = p_cov(1:9,1:9)*Ct'*(Ct*p_cov(1:9,1:9)*Ct' -  Wt*R*Wt')^-1;
        e_mean = p_mean(1:9) + Kt*(Z - h);
        e_cov = p_cov(1:9,1:9)- Kt*Ct*p_cov(1:9,1:9);
        Estimates(i,:) = e_mean';
        
        prev_mean = [e_mean;bg;ba];
        prev_cov = [e_cov zeros(9,6); zeros(3,9) eye(3)*0.01 zeros(3,3); zeros(3,12) eye(3)*0.01];
        prev_time = Curr_time;
        prev_am = am;
        prev_wm = wm;
    end
end



%%
scatter3(Estimates(:,1),Estimates(:,2), Estimates(:,3))
hold on;
scatter3(PoseiSAM2(:,1),PoseiSAM2(:,2), PoseiSAM2(:,3))