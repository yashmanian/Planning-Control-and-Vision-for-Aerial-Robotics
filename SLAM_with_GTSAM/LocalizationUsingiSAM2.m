function AllPosesComputed = LocalizationUsingiSAM2(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, LandMarksComputed)
addpath('../../../gtsam_toolbox')
import gtsam.*

% LandMarksComputed = load('LandMarksComputed.mat');
% LandMarksComputed = LandMarksComputed.LMcomputed;
% params = matfile('CalibParams.mat');
% data_ip = matfile('DataSlowCircle.mat');
% TagSize = params.TagSize;
% K = params.K; 
% DetAll = data_ip.DetAll;
%PoseiSAM2 = data_ip.PoseiSAM2;
%PoseGTSAM = data_ip.PoseGTSAM;
intK = K;


fx = K(1,1); fy = K(2,2); s = K(1,2); x0 = K(1,3); y0 = K(2,3);
K = Cal3_S2(fx,fy,s,x0,y0);

%% State keys
StateKeys = [];
for i = 1:size(DetAll,2) 
    StateKeys = [StateKeys ; symbol('x',i)];
end

%% Landmark keys
LMs = [];
for i = 1:size(DetAll,2)
    LMs = [LMs ; DetAll{i}(:,:)]; 
end
[~,uniqueIndexes] = unique(LMs(:,1));
tagIds = LMs(uniqueIndexes,:);


%% Landmark keys
LMkeys = uint64([]);
n = 1;
for a = 1:size(tagIds,1) 
    LMkeys(a,1) = tagIds(a,1);
    LMkeys(a,2) = symbol('p',n);
    n = n+1;
    LMkeys(a,3) = symbol('p',n);
    n = n+1;
    LMkeys(a,4) = symbol('p',n);
    n = n+1;
    LMkeys(a,5) = symbol('p',n);
    n = n+1;
end
%% Noise parameters
pointNoiseSigma = 0.00001;
poseNoi = .0000001;
poseNoiseSigmas = [poseNoi poseNoi poseNoi poseNoi poseNoi poseNoi]';
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
measurementNoiseSigma = 1;
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);

%% PnP for poses
poseTrans = {};
poseRot = {};
cameraParams = cameraParameters('IntrinsicMatrix', intK');

for a = 1:size(DetAll,2)
    CurrDet = DetAll{a};
    worldPoints = [];
    imagePoints = [];
    for j = 1:size(CurrDet,1)
            TagId = CurrDet(j,1);

            f = find(all(bsxfun(@eq, LandMarksComputed(:,1),TagId),2)); % get index of tag

            P1 = [LandMarksComputed(f,2) LandMarksComputed(f,3) 0];
            P2 = [P1(1)+TagSize P1(2) 0]; 
            P3 = [P1(1)+TagSize P1(2)+TagSize 0]; 
            P4 = [P1(1) P1(2)+TagSize 0]; 
            P = [P1;P2;P3;P4];

            imgPts = [CurrDet(j,2) CurrDet(j,3); CurrDet(j,4) CurrDet(j,5);CurrDet(j,6) CurrDet(j,7);CurrDet(j,8) CurrDet(j,9)];
            worldPoints  = [worldPoints ; P];
            imagePoints = [imagePoints ; imgPts];
    end
    
    % Return to later **
    if size(worldPoints,1) > 0
    [Rot, Trans] = estimateWorldCameraPose(imagePoints,worldPoints,cameraParams, 'MaxNumTrials' , 700, 'Confidence', 80,'MaxReprojectionError' , 3);                
    else
        Rot = PrevRot;
        Trans = PrevTrans;
    end
    poseTrans{a} = Trans;
    poseRot{a} = Rot;
    PrevRot = Rot;
    PrevTrans = Trans;
    R = rotm2eul(poseRot{a});
    T = poseTrans{a};
    truth.cameras{a} = SimpleCamera.Lookat(Point3([T(1),T(2),T(3)]'),Point3, Point3([R(1),R(2),R(3)]'), K);
end

%% Initialize isam2
parameters =  gtsam.ISAM2Params;
parameters.setOptimizationParams(ISAM2DoglegParams())

isam = ISAM2(parameters);
graph = NonlinearFactorGraph;
initialEstimate = Values;

%% Constraints
r1 = [TagSize 0 0]; 
r2 = [0 TagSize 0]; 
r3 = [-TagSize 0 0]; 
r4 = [0 -TagSize 0]; 
r5 = [TagSize TagSize 0]; 
r6 = [-TagSize TagSize 0]; 

for i = 1:2
    CurrDet = DetAll{i};
    CurrState = StateKeys(i);
    initialEstimate.insert(StateKeys(i),truth.cameras{i}.pose);
    
    if i == 1
        graph.add(PriorFactorPose3(StateKeys(1), truth.cameras{1}.pose, posePriorNoise));
    end

    for j = 1:size(CurrDet,1)
        TagId = CurrDet(j,1);
        Det = [CurrDet(j,2) CurrDet(j,3);CurrDet(j,4) CurrDet(j,5);CurrDet(j,6) CurrDet(j,7);CurrDet(j,8) CurrDet(j,9)];
        f = find(all(bsxfun(@eq, LMkeys(:,1),TagId),2));
        
        graph.add(GenericProjectionFactorCal3_S2(Point2(Det(1,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 2)),K));
        graph.add(GenericProjectionFactorCal3_S2(Point2(Det(2,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 3)),K));
        graph.add(GenericProjectionFactorCal3_S2(Point2(Det(3,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 4)),K));
        graph.add(GenericProjectionFactorCal3_S2(Point2(Det(4,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 5)),K));
        
        Corners = [LMkeys(f, 2) LMkeys(f,3) LMkeys(f, 4) LMkeys(f, 5)];
        
        P1 = [LandMarksComputed(f,2) LandMarksComputed(f,3) 0];
        P2 = [P1(1)+TagSize P1(2) 0]; 
        P3 = [P1(1)+TagSize P1(2)+TagSize 0]; 
        P4 = [P1(1) P1(2)+TagSize 0]; 
        P = [P1;P2;P3;P4];
        
        for k = 1:4
            if ~initialEstimate.exists(Corners(k))
                initialEstimate.insert(Corners(k), Point3((P(k,:))'));
                graph.add(PriorFactorPoint3(Corners(k), Point3(P(k,:)'), pointPriorNoise));
            end
        end
        
        graph.add(BetweenFactorPoint3(Corners(1),Corners(2),Point3(r1'),pointPriorNoise));
        graph.add(BetweenFactorPoint3(Corners(2),Corners(3),Point3(r2'),pointPriorNoise));
        graph.add(BetweenFactorPoint3(Corners(3),Corners(4),Point3(r3'),pointPriorNoise));
        graph.add(BetweenFactorPoint3(Corners(4),Corners(1),Point3(r4'),pointPriorNoise));
        graph.add(BetweenFactorPoint3(Corners(1),Corners(3),Point3(r5'),pointPriorNoise));
        graph.add(BetweenFactorPoint3(Corners(2),Corners(4),Point3(r6'),pointPriorNoise));
    end
    
    if i == 2
        graph.add(BetweenFactorPose3(StateKeys(i-1),StateKeys(i),Pose3(Rot3(eye(3)),Point3([0 0 0]')),posePriorNoise));
    end
end

%% Initial update
batchOptimizer = DoglegOptimizer(graph,initialEstimate);
fullyOptimized = batchOptimizer.optimize();
isam.update(graph, fullyOptimized);
result = isam.calculateEstimate();

AllPosesComputed=[];
for i= 1:2
    quat = result.at(symbol('x',i)).rotation.quaternion;
    quat = quat';
    AllPosesComputed=[AllPosesComputed;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z, quat];
end


%% ISAM
for i = 3:size(DetAll,2)
    graph = NonlinearFactorGraph;
    initialEstimate = Values;
    CurrDet = DetAll{i}; 
    CurrState = StateKeys(i);
    
    r_odom = poseRot{i}-poseRot{i-1}; 
    p_odom = poseTrans{i}-poseTrans{i-1};
    graph.add(BetweenFactorPose3(StateKeys(i-1),StateKeys(i),Pose3(Rot3(r_odom),Point3(p_odom')),posePriorNoise));
    
    for j = 1:size(CurrDet,1)
            TagId = CurrDet(j,1);
            f = find(all(bsxfun(@eq, LMkeys(:,1),TagId),2));
            Det = [CurrDet(j,2) CurrDet(j,3);CurrDet(j,4) CurrDet(j,5);CurrDet(j,6) CurrDet(j,7);CurrDet(j,8) CurrDet(j,9)];
            
            Corners = [LMkeys(f, 2) LMkeys(f,3) LMkeys(f, 4) LMkeys(f, 5)];
            
            graph.add(GenericProjectionFactorCal3_S2(Point2(Det(1,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 2)),K));
            graph.add(GenericProjectionFactorCal3_S2(Point2(Det(2,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 3)),K));
            graph.add(GenericProjectionFactorCal3_S2(Point2(Det(3,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 4)),K));
            graph.add(GenericProjectionFactorCal3_S2(Point2(Det(4,:)'),measurementNoise, CurrState, uint64(LMkeys(f, 5)),K));

            graph.add(BetweenFactorPoint3(Corners(1),Corners(2),Point3(r1'),pointPriorNoise));
            graph.add(BetweenFactorPoint3(Corners(2),Corners(3),Point3(r2'),pointPriorNoise));
            graph.add(BetweenFactorPoint3(Corners(3),Corners(4),Point3(r3'),pointPriorNoise));
            graph.add(BetweenFactorPoint3(Corners(4),Corners(1),Point3(r4'),pointPriorNoise));
            graph.add(BetweenFactorPoint3(Corners(1),Corners(3),Point3(r5'),pointPriorNoise));
            graph.add(BetweenFactorPoint3(Corners(2),Corners(4),Point3(r6'),pointPriorNoise));
            
            P1 = [LandMarksComputed(f,2) LandMarksComputed(f,3) 0];  
            P2 = [P1(1) + TagSize  P1(2) 0];
            P3 = [P1(1) + TagSize P1(2) + TagSize 0];
            P4 = [P1(1) P1(2) + TagSize 0];
            P = [P1 ; P2 ; P3 ; P4];  

            for k = 1:4
                if ~initialEstimate.exists(Corners(k)) && ~result.exists(Corners(k))
                    initialEstimate.insert(Corners(k), Point3((P(k,:))'));
                    graph.add(PriorFactorPoint3(Corners(k), Point3(P(k,:)'), pointPriorNoise));
                end
            end
    end
    
    initialEstimate.insert(StateKeys(i),truth.cameras{i}.pose);

    isam.update(graph,initialEstimate);
    result = isam.calculateEstimate();
    
    quat = result.at(symbol('x',i)).rotation.quaternion;
    quat = quat';
    AllPosesComputed=[AllPosesComputed;result.at(symbol('x',i)).x, result.at(symbol('x',i)).y,result.at(symbol('x',i)).z, quat];
end

end

% scatter3(AllPosesComputed(:,1),AllPosesComputed(:,2),AllPosesComputed(:,3));
% hold on
% scatter(LandMarksComputed(:,2),LandMarksComputed(:,3),'filled')
% 
% % With isam
% resultIsam = isam.calculateBestEstimate();
% stateResultsIsam = [];
% for n = 1:size(StateKeys,1)
%     stateResultsIsam = [stateResultsIsam ; [resultIsam.at(StateKeys(n)).x resultIsam.at(StateKeys(n)).y resultIsam.at(StateKeys(n)).z]];
% end  
% 
% % stateResults = stateResultsIsam;
% stateResults = AllPosesComputed;
% for i = 1:100
%     for n = 1:size(stateResults,1)
%         xpos = stateResults(n,1);
%         ypos = stateResults(n,2);
%         zpos = stateResults(n,3);
%         if n > 1
%            %dist = ( (xpos-xprev)^2 + (ypos-yprev)^2 + (zpos-zprev)^2 )^.5;
%             if abs(xpos-xprev) > 1 || abs(ypos-yprev) > 1 || abs(zpos-zprev) > 1
%            %     [dist xpos ypos zpos]
%                 stateResults(n,1) = xprev;
%                 stateResults(n,2) = yprev;
%                 stateResults(n,3) = zprev;
%             end
% 
%         end
%         xprev = xpos;
%         yprev = ypos;
%         zprev = zpos;
%     end
% end

%figure
%scatter3(stateResultsIsam(:,1),stateResultsIsam(:,2),stateResultsIsam(:,3))
% scatter3(PoseiSAM2(:,1),PoseiSAM2(:,2),PoseiSAM2(:,3))
% hold on
% scatter(LandMarksComputed(:,2),LandMarksComputed(:,3),'filled')
% title('Poses with Isam2')

%%
% EstPos = AllPosesComputed(:, 1:3);
% TruthPos = PoseiSAM2(:, 1:3);
% PosError = ComputePosError(EstPos, TruthPos);
% 
% EstAng = AllPosesComputed(:, 4:7)';
% TruthAng = PoseiSAM2(:, 4:7)';
% AngError = ComputeAngError(EstAng, TruthAng);
% 
% figure
% plot(1:size(AllPosesComputed), PosError)
% hold on
% plot(1:size(AllPosesComputed), AngError)
% legend('Position Error','Angular Error')
