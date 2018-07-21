function [desired_state] = pos(t, qn)
% DIAMOND trajectory generator for a diamond
% DIAMOND IS UNBREAKABLE
% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
if t == 0
    pos = [0 0 0];
    vel = [0 0 0];
    acc = [0 0 0];
else
    pos = [0.5 0.5 2];
    vel = [0 0 0];
    acc = [0 0 0];
end
yaw = 0;
yawdot = 0;
% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
