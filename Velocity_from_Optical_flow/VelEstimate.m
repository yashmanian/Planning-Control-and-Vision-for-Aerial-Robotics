clear all
clc

ImageFolder = 'Images/StraightLineFrames';
data = matfile('DataStraightLine.mat');
data1 = load('PosesComputed_StraightLine.mat');
params = matfile('CalibParams.mat');

ImageList = ls(ImageFolder);
ImageList(1:2,:) = [];

TagSize = params.TagSize;
K = params.K; 
DetAll = data.DetAll;
TLeftImgs = data.TLeftImgs;
VelGTSAM = data.VelGTSAM;
VeliSAM2 = data.VeliSAM2;
cameraParams = cameraParameters('IntrinsicMatrix', K');


LandMarks = data1.LandMarksComputed;
Poses = data1.AllPosesComputed;
Velx = zeros(size(ImageList,1),1);
Vely = zeros(size(ImageList,1),1);
Velz = zeros(size(ImageList,1),1);

prev_corners = [];
prev_time = [];

%%
for f = 1:size(ImageList,1)
    if isempty(prev_time)
        delta_t = 0.0667;
    else
        delta_t = (TLeftImgs(f) - prev_time);
    end

    if isempty(prev_corners)
        filename = strcat(ImageFolder,'/', num2str(f),'.jpg');
        prev_time = TLeftImgs(f);
        
        CurrDet = DetAll{f};
        if isempty(CurrDet)
            velocityXYZ = [0;0;0];
            prev_time = TLeftImgs(f);
            prev_corners = [];
            PrevImg = [];
            Velx(f) = velocityXYZ(1);
            Vely(f) = velocityXYZ(2);
            Velz(f) = velocityXYZ(3);
            continue;
        end
        TagId = CurrDet(1,1);
        PrevImg = rgb2gray(imread(filename));
        corners = detectFASTFeatures(PrevImg);
        prev_corners = corners.selectStrongest(800).Location;
    else
        filename = strcat(ImageFolder,'/', num2str(f),'.jpg');
        delta_t = TLeftImgs(f)- TLeftImgs(f-1);
        
        CurrDet = DetAll{f};
        if isempty(CurrDet)
            velocityXYZ = [0;0;0];
            prev_time = TLeftImgs(f);
            prev_corners = [];
            PrevImg = [];
            Velx(f) = velocityXYZ(1);
            Vely(f) = velocityXYZ(2);
            Velz(f) = velocityXYZ(3);
            continue;
        end
        TagId = CurrDet(1,1);
        CurrImg = rgb2gray(imread(filename));

%% KLT tracking
        pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
        oldPoints = prev_corners;
        initialize(pointTracker, oldPoints, PrevImg);
        [points, isFound] = step(pointTracker, CurrImg);
%%  Convert to camera frame
        visiblePoints = points(isFound, :);
        oldInliers = oldPoints(isFound, :);
        
        points = [visiblePoints ones(size(visiblePoints,1),1)];
        points_cf = K\points';
        points_prev = [oldInliers ones(size(visiblePoints,1),1)];
        points_cf_prev = K\points_prev';
        points_cf = points_cf';
        points_cf_prev = points_cf_prev';
        
        m = find(all(bsxfun(@eq, LandMarks(:,1),TagId),2));
        imagePoints = [CurrDet(1,2) CurrDet(1,3); CurrDet(1,4) CurrDet(1,5); CurrDet(1,6) CurrDet(1,7); CurrDet(1,8) CurrDet(1,9)];
        worldPoints = [LandMarks(m,2) LandMarks(m,3); LandMarks(m,4) LandMarks(m,5); LandMarks(m,6) LandMarks(m,7); LandMarks(m,8) LandMarks(m,9)];
%%  Calculate Velocity
        [velocityXYZ,flow_vect] = estimateVelocity(delta_t, points_cf, points_cf_prev, imagePoints, worldPoints, cameraParams);
        
        if isempty(prev_corners)
            velocityXYZ = [0;0;0];
        end
        
        Velx(f) = velocityXYZ(1);
        Vely(f) = velocityXYZ(2);
        Velz(f) = velocityXYZ(3);
        PrevImg = CurrImg;
        corners = detectFASTFeatures(PrevImg);
        prev_corners = corners.selectStrongest(800).Location;
        prev_time = TLeftImgs(f);
    end
end

%%
EstVel = [Velx Vely Velz];
TruthVel = VeliSAM2;

VelError = ComputeVelError(EstVel, TruthVel);


%%
figure
subplot(3,1,1)
plot(1:size(Velx,1), Velx)
hold on;
plot(1:size(Vely,1), VelGTSAM(:,1))
axis([0 size(Vely,1) -0.5 0.5])
legend('Estimated velocity','VeliSAM2')
title('Estimated velocity vs Ground truth X-axis')

subplot(3,1,2)
plot(1:size(Velx,1), Vely)
hold on;
plot(1:size(Vely,1), VelGTSAM(:,2))
axis([0 size(Vely,1) -0.5 0.5])
legend('Estimated velocity','VeliSAM2')
title('Estimated velocity vs Ground truth Y-axis')


subplot(3,1,3)
plot(1:size(Velz,1), Velz)
hold on;
plot(1:size(Vely,1), VelGTSAM(:,3))
axis([0 size(Vely,1) -0.5 0.5])
legend('Estimated velocity','VeliSAM2')
title('Estimated velocity vs Ground truth Z-axis')


figure
subplot(3,1,1)
plot(1:size(Velx,1), VelError(:,1))
axis([0 size(Vely,1) -0.1 0.5])
title('Velocity Error X-axis')

subplot(3,1,2)
plot(1:size(Velx,1), VelError(:,2))
axis([0 size(Vely,1) -0.1 0.5])
title('Velocity Error Y-axis')

subplot(3,1,3)
plot(1:size(Velz,1), VelError(:,3))
axis([0 size(Vely,1) -0.1 0.5])
title('Velocity Error Z-axis')

%%
figure
imshow(CurrImg)
hold on;
quiver(visiblePoints(:,1), visiblePoints(:,2), flow_vect(:,1), flow_vect(:,2), 'r')