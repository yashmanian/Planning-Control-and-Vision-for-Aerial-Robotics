function [LMcomputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                               IMU, LeftImgs, TLeftImgs, Mode)

%function [LMcomputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize)
% For Input and Output specifications refer to the project pdf
% clear all
% close all
% clc
import gtsam.*

%% Import
% params = matfile('CalibParams.mat');
% data = matfile('DataFastCircle.mat');
% TagSize = params.TagSize;
% K = params.K; 
%DetAll = data.DetAll;

intK = K;
fx = K(1,1); 
fy = K(2,2); 
s = K(1,2); 
x0 = K(1,3); 
y0 = K(2,3);

DetAll = DetAll(~cellfun('isempty',DetAll));

K = Cal3_S2(fx,fy,s,x0,y0);
%% Add factor graph
graph = NonlinearFactorGraph;
%% State keys
StateKeys = [];
for a = 1:size(DetAll,2)
    truth.cameras{a} = SimpleCamera.Lookat(Point3([1,1,1]'), Point3, Point3([0,0,1]'), K);
    StateKeys = [StateKeys ; symbol('x',a)];
end
%% AR Tag Landmarks
LMs = [];
for a = 1:size(DetAll,2)
    LMs = [LMs ; DetAll{a}(:,:)]; 
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

%% Projective Measurement Noise
measurementNoiseSigma = 1.0;
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);

%% Homography setup
Neighbors = []; 
n = 1+TagSize;
TagRef = [1 1; n 1; n n; 1 n];

HpointsRef = [TagRef'; 1 1 1 1];


for a = 1:size(DetAll,2) 
    CurrDet = DetAll{a}; 
    CurrState = StateKeys(a); 
    globalInFrame = false;
    if a == 1 
        if size(find(all(bsxfun(@eq,CurrDet(:,1),10),2)),1) > 0 
            globalId = 10;
        else
            globalId = CurrDet(1,1); 
        end
        Neighbors = [globalId 0 0];
    end
    
    if size(find(all(bsxfun(@eq,CurrDet(:,1),globalId),2)),1) > 0 
        globalInFrame = true;
    end
    
    if globalInFrame 
        for b = 1:size(CurrDet,1)
            CurrID = CurrDet(b,1); 
            
            if CurrID == globalId 
                 globalP1 = [DetAll{a}(b,2) DetAll{a}(b,3) 1; DetAll{a}(b,4) DetAll{a}(b,5) 1; DetAll{a}(b,6) DetAll{a}(b,7) 1; DetAll{a}(b,8) DetAll{a}(b,9) 1]';
                 H = homography2d(globalP1,HpointsRef); 

                 for i = 1:size(CurrDet,1)          
                     if CurrDet(i,1) ~= globalId 
                         if size(find(all(bsxfun(@eq,Neighbors(:,1),CurrDet(i,1)),2)),1) < 1 
                             tagPoints = [DetAll{a}(i,2) DetAll{a}(i,3) ];
                             oldPoint = tagPoints;
                             oldPoint(:,3) = 1;
                             oldPoint = oldPoint';
                             oldPoint = oldPoint(:,1);

                             newPoint = H*oldPoint;
                             newPoint = newPoint/newPoint(3);
                             delta_x = (newPoint(1) - 1)/10; 
                             delta_y = (newPoint(2) - 1)/10;
                             scatter(delta_x,delta_y,'b','filled')
                             Neighbors = [Neighbors ; [CurrDet(i,1) delta_x delta_y] ];
                         end
                     end
                 end
            end
            
            f = find(all(bsxfun(@eq, LMkeys(:,1),CurrID),2));
            measuredPoint1 = Point2([CurrDet(b,2) CurrDet(b,3)]');
            measuredPoint2 = Point2([CurrDet(b,4) CurrDet(b,5)]');
            measuredPoint3 = Point2([CurrDet(b,6) CurrDet(b,7)]');
            measuredPoint4 = Point2([CurrDet(b,8) CurrDet(b,9)]');
            
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint1,measurementNoise, CurrState, LMkeys(f,2),K));
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint2,measurementNoise, CurrState, LMkeys(f,3),K));
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint3,measurementNoise, CurrState, LMkeys(f,4),K));
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint4,measurementNoise, CurrState, LMkeys(f,5),K));
        end
    end
    
    if ~globalInFrame
        for i = 1:size(CurrDet,1) 
            if size(find(all(bsxfun(@eq, Neighbors(:,1) , CurrDet(i,1)),2)),1) > 0 
                globalReference = CurrDet(i,1); 
                break
            end
        end
        
        for b = 1:size(CurrDet,1)
            CurrID = CurrDet(b,1);

            if CurrID == globalReference 
                 globalP1 = [DetAll{a}(b,2) DetAll{a}(b,3) 1; DetAll{a}(b,4) DetAll{a}(b,5) 1; DetAll{a}(b,6) DetAll{a}(b,7) 1; DetAll{a}(b,8) DetAll{a}(b,9) 1]';
                 H = homography2d(globalP1,HpointsRef); 
                 H = H/H(3,3);

                 for i = 1:size(CurrDet,1)  
                     if CurrDet(i,1) ~= globalReference 

                         if size(find(all(bsxfun(@eq,Neighbors(:,1),CurrDet(i,1)),2)),1) < 1 
                             tagPoints = [DetAll{a}(i,2) DetAll{a}(i,3) ]; 
                
                             oldPoint = tagPoints;
                             oldPoint(:,3) = 1;
                             oldPoint = oldPoint';
                             oldPoint = oldPoint(:,1);

                             newPoint = H*oldPoint;
                             newPoint = newPoint/newPoint(3);
                             
                             globalReferenceIndex = find(all(bsxfun(@eq,Neighbors(:,1),globalReference),2));
                             delta_x_ref = Neighbors(globalReferenceIndex,2);
                             delta_y_ref = Neighbors(globalReferenceIndex,3);
                             
                             delta_x = (newPoint(1) - 1)/10;
                             delta_y = (newPoint(2) - 1)/10;
                             
                             delta_x = delta_x + delta_x_ref;
                             delta_y = delta_y + delta_y_ref;
                             
                             scatter(delta_x,delta_y,'b','filled')
                             Neighbors = [Neighbors ; [CurrDet(i,1) delta_x delta_y] ];
                         end
                     end
                 end
            end
            
            f = find(all(bsxfun(@eq, LMkeys(:,1),CurrID),2));
            measuredPoint1 = Point2([CurrDet(b,2) CurrDet(b,3)]');
            measuredPoint2 = Point2([CurrDet(b,4) CurrDet(b,5)]');
            measuredPoint3 = Point2([CurrDet(b,6) CurrDet(b,7)]');
            measuredPoint4 = Point2([CurrDet(b,8) CurrDet(b,9)]');
            
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint1,measurementNoise, CurrState, LMkeys(f,2),K));
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint2,measurementNoise, CurrState, LMkeys(f,3),K));
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint3,measurementNoise, CurrState, LMkeys(f,4),K));
            graph.add(GenericProjectionFactorCal3_S2(measuredPoint4,measurementNoise, CurrState, LMkeys(f,5),K));
        end
            
    end

end

%% Noise parameters
pointNoiseSigma = 0.00001;
poseNoiseSigmas = [0.01 0.01 0.01 0.01 0.01 0.01]';
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);

%% Applying Constraints
r1 = [TagSize 0 0]; 
r2 = [0 TagSize 0]; 
r3 = [-TagSize 0 0]; 
r4 = [0 -TagSize 0]; 
r5 = [TagSize TagSize 0]; 
r6 = [-TagSize TagSize 0]; 
for a = 1:size(LMkeys,1)
    graph.add(BetweenFactorPoint3(LMkeys(a,2),LMkeys(a,3),Point3(r1'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(LMkeys(a,3),LMkeys(a,4),Point3(r2'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(LMkeys(a,4),LMkeys(a,5),Point3(r3'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(LMkeys(a,5),LMkeys(a,2),Point3(r4'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(LMkeys(a,2),LMkeys(a,4),Point3(r5'),pointPriorNoise));
    graph.add(BetweenFactorPoint3(LMkeys(a,3),LMkeys(a,5),Point3(r6'),pointPriorNoise));
end   

%% Add priors
graph.add(PriorFactorPose3(StateKeys(1), truth.cameras{1}.pose, posePriorNoise));

graph.add(PriorFactorPoint3(LMkeys(1,2), Point3(0,0,0), pointPriorNoise));
graph.add(PriorFactorPoint3(LMkeys(1,3), Point3([TagSize 0 0]'), pointPriorNoise));
graph.add(PriorFactorPoint3(LMkeys(1,4), Point3([TagSize TagSize 0]'), pointPriorNoise));
graph.add(PriorFactorPoint3(LMkeys(1,5), Point3([0 TagSize 0]'), pointPriorNoise));

%% Initial Estimates
initialEstimate = Values;
for a = 1:size(StateKeys,1)
%     rot = eul2rotm([rand rand rand]);
%     trans = [rand rand rand]';
    estimatePose = truth.cameras{a}.pose.retract(0.1*randn(6,1));
    initialEstimate.insert(StateKeys(a),estimatePose);
    if a <= size(StateKeys,1)-1
        graph.add(BetweenFactorPose3(StateKeys(a),StateKeys(a+1),Pose3(Rot3(eye(3)),Point3([0 0 0]')),posePriorNoise));
    end
end

for a = 1:size(LMkeys,1)
    tagId = LMkeys(a,1);
    NeighborIndex = find(all(bsxfun(@eq, Neighbors(:,1),tagId),2));
    Pt1X = Neighbors(NeighborIndex,2);
    Pt1Y = Neighbors(NeighborIndex,3); 
    
    initialEstimate.insert(uint64(LMkeys(a,2)), Point3([Pt1X Pt1Y 0]'));
    initialEstimate.insert(uint64(LMkeys(a,3)), Point3([Pt1X+TagSize Pt1Y 0]'));
    initialEstimate.insert(uint64(LMkeys(a,4)), Point3([Pt1X+TagSize Pt1Y+TagSize 0]'));
    initialEstimate.insert(uint64(LMkeys(a,5)), Point3([Pt1X Pt1Y+TagSize 0]'));
end


%% Optimize
optimizer = DoglegOptimizer(graph,initialEstimate);
result = optimizer.optimize();


States = [];
for n = 1:size(StateKeys,1)
    quat = result.at(symbol('x',n)).rotation.quaternion;
    quat = quat';
    States = [States ; [result.at(StateKeys(n)).x result.at(StateKeys(n)).y result.at(StateKeys(n)).z, quat]];
end

LMres = [];
LMcenters = [];
LMcomputed = [];
for n = 1:size(LMkeys,1)
    LMcomputed = [LMcomputed ;[tagIds(n,1) result.at(LMkeys(n,2)).x result.at(LMkeys(n,2)).y result.at(LMkeys(n,3)).x result.at(LMkeys(n,3)).y result.at(LMkeys(n,4)).x result.at(LMkeys(n,4)).y result.at(LMkeys(n,5)).x result.at(LMkeys(n,5)).y]];                                 
    xc = result.at(LMkeys(n,2)).x ;
    yc = result.at(LMkeys(n,2)).y ;
    LMcenters(n,:) = [xc yc];
end

figure
scatter3(LMcomputed(:,2),LMcomputed(:,3), zeros(size(LMcomputed,1),1),'filled')
hold on
scatter3(States(:,1),States(:,2),States(:,3))
AllPosesComputed = States;
%%
save('LandMarksComputed','LMcomputed');

end