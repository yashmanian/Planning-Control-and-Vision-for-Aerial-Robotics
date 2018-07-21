function [velocityXYZ,flow_vect] = estimateVelocity(delta_t, points_cf, points_cf_prev, imagePoints, worldPoints, cameraParams)
flow_vect = (points_cf - points_cf_prev)/delta_t;

[Rot,Trans] = extrinsics(imagePoints,worldPoints,cameraParams);
Rot = Rot';
Rt = [Rot(:,1) Rot(:,2) Trans'];

points_size = size(points_cf,1);
flow = zeros(2 * points_size, 1);
F = zeros(2 * points_size, 6);
points_cf = points_cf';
points_cf_prev = points_cf_prev';
c = 1;
for i = 1:points_size
    flow(c,:) = flow_vect(i,1);
    flow(c+1,:) = flow_vect(i,2);
    cornerWorld = Rt \ points_cf(:,i);
    x = points_cf(1,i);
    y = points_cf(2,i);
    Z = 1 / cornerWorld(3);

    F1 = [-1/Z, 0, x/Z, x*y, -(1+x^2), y];
    F2 = [0, -1/Z, y/Z, (1+y)^2, -x*y, -x];
    F(c,:) = F1;
    F(c+1,:) = F2;

    c = c + 2;
end

velocities = F \ flow;
velocityXYZ = Rot' * velocities(1:3);
end