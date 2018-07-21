function updatedState = runEKF(params,data,poses,velocities,getJacobianA,getJacobianU)
%!synclient HorizEdgeScroll=0 HorizTwoFingerScroll=0
tic
% Load parameters and data
%params = matfile('CalibParams.mat');
%data = matfile('DataStraightLine.mat'); % <------------------------------- Set the data 

accelerations = []; % accelerations from IMU
angVelocities = []; % rotational Velocities from IMU

%velocities = data.VeliSAM2; % [Vx,Vy,Vz] from optical flow i think
%poses = data.PoseiSAM2; % [PosX, PosY, PosZ, QuaternionW, QuaternionX, QuaternionY, QuaternionZ] 

% Match data times and imu times
dataTime = data.TLeftImgs; % times from Det all frames
imuTime = data.IMU(:,11); % times from IMU
badIndex = []; % indexes from dataTime to remove
imuGoodTimes = []; % var to hold IMU data at good times
for i = 1:size(dataTime) % loop through data times
    
    differences = abs(dataTime(i) - imuTime); % get difference between current data time and all imutimes
    [minDiff,j] = min(differences); % get minimum time difference and index in IMU of minimum difference
    if minDiff > .3 ||( poses(i,1) == 0 && poses(i,2) == 0 && poses(i,3) == 0 && poses(i,4) ==0 && poses(i,5) == 0) % if the minimum is greater than __, times dont match up, will not use that frame
        badIndex = [badIndex ; i]; % add bad index
        
    elseif minDiff <= .3
        imuGoodTimes = [imuGoodTimes ; imuTime(j) ]; % add imu time for the matched IMU row
        accelerations = [accelerations ; data.IMU(j,5:7)]; % add acceleration term for matched frame
        angVelocities = [angVelocities ; data.IMU(j,8:10)]; % add angular velocity term for matched frame
    end
end

% Delete rows that do not align
dataTime(badIndex,:) = [];
velocities(badIndex,:) = [];
poses(badIndex,:) = [];

% get positions and quaterinons from isam2
positions = poses(:,1:3); % [PosX PosY PosZ]
quaternions = poses(:,4:7); % [QuaternionW, QuaternionX, QuaternionY, QuaternionZ]
angles = quat2eul(quaternions);
yaw = angles(1); % angles from isam, used in measurement
pitch = angles(2);
roll = angles(3);

% Transform quaternion IMU
quatIMU_Transform = params.qIMUToC;
qin = quatIMU_Transform';
% Convert quaternion translat to roll pitch yaw, add tranformation
r1 = 2.*(qin(:,2).*qin(:,3) + qin(:,1).*qin(:,4));
r2 = qin(:,1).^2 + qin(:,2).^2 - qin(:,3).^2 - qin(:,4).^2;
r3 = -2.*(qin(:,2).*qin(:,4) - qin(:,1).*qin(:,3));
r4 = 2.*(qin(:,3).*qin(:,4) + qin(:,1).*qin(:,2));
r5 = qin(:,1).^2 - qin(:,2).^2 - qin(:,3).^2 + qin(:,4).^2;
yawTransform = atan2(r1,r2);
pitchTransform = asin(r3);
rollTransform = atan2(r4, r5);

% get rotation of IMU to camera
rotIMU = quat2rotm(quatIMU_Transform');

translateIMU = params.TIMUToC;
% Transform IMU angles
for c = 1:size(angVelocities,1)
    angV = rotIMU*angVelocities(c,:)' + [translateIMU(1) translateIMU(2) translateIMU(3)]';
    angVelocities(c,:) = angV'; 
end

%angVelocities + [yawTranslate pitchTranslate rollTranslate];

% Transform IMU linear accelerations
translateIMU = params.TIMUToC;
% accelerations = accelerations + [translateIMU(1) translateIMU(2) translateIMU(3)]; % apply linear transformation
for b = 1:size(accelerations,1)
    acc = rotIMU*accelerations(b,:)' + [translateIMU(1) translateIMU(2) translateIMU(3)]'; % apply trans/rotational transformation
    accelerations(b,:) = acc';
end

% Get Variables
K = params.K;
DetAll = data.DetAll; % data at each detection
gravity = [0 0  -9.807]'; % gravity
toc


for i = 1:size(positions,1) % loop
% If first state use measurement values for state
    if i == 1 % how to initialize?
        p = positions(i,:)'; % position
        pdot = velocities(i,:)'; % velocity of center of mass
        yaw = angles(i,1);  psi = yaw;
        pitch = angles(i,2); theta = pitch;
        roll = angles(i,3); phi = roll;
        q = [roll pitch yaw]';
        bg = .0000000000001*[1;1;1]; % ang velocity bias
        ba = .0000000000001*[1;1;1]; % acceleration bias
       % omega_prev = angVelocities(i,:)'; % angular speeds, from phase 2
       % omega_m_prev = omega_prev + bg + ng; % angular speeds + noise and bias
        %Xdot = [pdot ; G \ (omega_m_prev - bg - ng) ;  gravity +
        %R_q*(a_m_prev-ba-na) ; nbg ; nba ]; Just make zero? maybe idk
        updatedState(i,:) = [p'  q'  pdot'  bg'  ba'];
        covariances{i} = zeros(15);
        %updatedXdot(i,:) = Xdot'; % keep track of xdots?

        continue
    end

    %% PROCESS MODEL
    % x,y,z and Vx, Vy, Vz
    tic
    p_prev = updatedState(i-1,1:3)'; % position
    pdot_prev = updatedState(i-1,7:9)'; % velocity of center of mass

    % psi = yaw, theta = pitch, phi = roll
    q_prev = updatedState(i-1,4:6)';
    yaw = q(3);  psi = yaw;
    pitch = q(2); theta = pitch;
    roll = q(1); phi = roll;

    % Biases and noise
    bg = .1*[1;1;2]; % ang velocity bias
    %bg = updatedState(i-1,10:12)';
    ng = bg; % measurement noise of ang velocity
    %ng = random('norm',0,0.01,3,1);
    nbg = .00000000000001*[1 1 1]';%bg*1; % mean 0, covariance Qg
    ba = .1*[1;1;2]; % acceleration bias
    %ba = updatedState(i-1,13:15)';
    %na = random('norm',0,0.01,3,1);
   na = ba; % measurement noise of acceleration
    nba = .00000000000001*[1 1 1]';%ba *1; % mean 0, caovariance Qa

    % Rotation matrix
    R_q_Row1 = [cos(psi)*cos(theta)-sin(phi)*sin(psi)*cos(theta)    -cos(phi)*sin(psi)     cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi)];
    R_q_Row2 = [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)   cos(phi)*cos(psi)     sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi)];
    R_q_Row3 = [            -cos(phi)*sin(theta)                          sin(phi)                       cos(phi)*cos(theta)               ];
    R_q = [R_q_Row1 ; R_q_Row2 ; R_q_Row3];

    % Coefficient matrix
    G = [cos(q(2))    0    -cos(q(1))*sin(q(2));...
             0        1           sin(q(1));...
         sin(q(2))    0     cos(q(1))*cos(q(2))];

    % Angular speed and Acceleration from IMU
    omega_prev = angVelocities(i-1,:)'; % angular speeds, from imu
    omega_m_prev = omega_prev + bg + ng; % angular speeds + noise and bias
    a_m_prev = accelerations(i-1,:)'; % accelerometer measurement, from imu

    % Time difference
    timeDiff = dataTime(i)-dataTime(i-1);

    % Compute state X and Xdot for prediction of state
    X_prev = [ p_prev ; q_prev ; pdot_prev ; bg ; ba];
    Xdot_prev = [pdot_prev ; G \ (omega_m_prev - bg - ng) ;  gravity + R_q*(a_m_prev-ba-na) ; nbg ; nba ];

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
    R(1,1) = Rp; R(2,2) = Rp; R(3,3) = Rp;
    R(4,4) = Rq; R(5,5) = Rq; R(6,6) = Rq;
    R(7,7) = Rpdot; R(8,8) = Rpdot; R(9,9) = Rpdot;
    R(10,10) = Rn; R(11,11) = Rn ; R(12,12) = Rn;
    R(13,13) = Rn; R(14,14) = Rn; R(15,15) = Rn;
    

    % partial f / partial x jacobian
    A = getJacobianA(a_m_prev(1),a_m_prev(2),a_m_prev(3),ba(1),ba(2),ba(3),bg(1),bg(2), bg(3),na(1),na(2),na(3),ng(1),ng(2),ng(3)...
                      ,phi,psi,theta,omega_m_prev(1),omega_m_prev(2),omega_m_prev(3)); 

    % partial f / partial noise jacobian
    U = getJacobianU(phi,psi,theta);    
    Ft = eye(15) + A*timeDiff;
    Vt = U*timeDiff;

    % Previous covariance P
    P_prev = covariances{i-1};

    % Prediction Step
    %Xdot_prev(9) = Xdot_prev(9)/20;
    Xpredict = X_prev + Xdot_prev*timeDiff;
    Ppredict = Ft*P_prev*Ft' + Vt*Q*Vt';
    xfoundPredicted(i,:) = [Xpredict']; % keep track of states found just from prediction


    %toc
    %tic
    %% MEASUREMENT MODEL
    % partial h / partial x will just give identity since z = [p ; q ; pdot] and thats first 3 entries of x
    C = [eye(3) zeros(3)  zeros(3) zeros(3) zeros(3);
         zeros(3) eye(3)  zeros(3) zeros(3) zeros(3);
         zeros(3) zeros(3) eye(3) zeros(3) zeros(3)];

    % Measured quantities
    % x,y,z and Vx, Vy, Vz
    p = positions(i,:)'; % position
    pdot = velocities(i,:)'; % velocity of center of mass

    % psi = yaw, theta = pitch, phi = roll
    yaw = angles(i,1);  psi = yaw;
    pitch = angles(i,2); theta = pitch;
    roll = angles(i,3); phi = roll;
    q = [roll pitch yaw]';

    % measurement model
    z = [p ; q ; pdot];
    %zt = [z ; 0; 0; 0; 0; 0; 0];
    % y = z - [Xpredict(1:9)];


    % Get jacobian W
    %W = getJacobianW();
    W = .5*[eye(3) zeros(3)  zeros(3) zeros(3) zeros(3);
         zeros(3) eye(3)  zeros(3) zeros(3) zeros(3);
         zeros(3) zeros(3) eye(3) zeros(3) zeros(3)];

    % Get Kalman Gain
    Kg = Ppredict*C' * (C*Ppredict*C' + W*R*W')^-1;

    % Update step
    X_updated = Xpredict + Kg*(z-Xpredict(1:9));
    covariance_updated = Ppredict -Kg*C*Ppredict;

    % Add to list of final states/final covariances
    updatedState(i,:) = X_updated';
    covariances{i} = covariance_updated;
    toc


    % FInding noisemodel
    % SlamDunk website get biases from them, can start at 0
    % Already have noise R from isam, make Q order magnitude higher
    % check if code is working, set Q huge which will give you back measuremnts
    % so gives you back measurements you gave in, it should match, good way to
    % check your code

end

% 5;
% pf = robotics.ParticleFilter;
% pf.StateEstimationMethod = 'mean';
% pf.ResamplingMethod = 'residual';
% %pf.StateTransitionFunction = 
% initialize(pf,10000,[.1398 -.67 1.05],eye(3));
% robotPred = zeros(length(updatedState),3);
% robotCorrected = zeros(length(updatedState),3);
% 
% for i = 1:length(updatedState)
%     % Predict next position. Resample particles if necessary.
%     [robotPred(i,:),robotCov] = predict(pf);
%     % observation step.
%     measurement(i,:) = positions(i,:);
%     % Correct position based on the given measurement to get best estimation.
%     [robotCorrected(i,:),robotCov] = correct(pf,measurement(i,:));
% end
% 
% 5;
% end
% plot em
% updatedPos = [updatedState(:,1) updatedState(:,2) updatedState(:,3)];
% plot3(updatedPos(:,1),updatedPos(:,2),updatedPos(:,3),'b'); hold on 
% plot3(positions(:,1),positions(:,2),positions(:,3),'r')
% title('Paths')
% legend('Filter output','Measured')
% box on
% 
% figure
% error = abs(updatedState(:,1:3) - positions);
% plot(error(:,1)) 
% hold on
% plot(error(:,2))
% plot(error(:,3))
% title('Difference between measured')
% legend('x','y','z')
% 
% figure
% xfoundPredicted(1,:) = xfoundPredicted(2,:);
% plot3(xfoundPredicted(:,1),xfoundPredicted(:,2),xfoundPredicted(:,3))
% title('predicted path')
% box on