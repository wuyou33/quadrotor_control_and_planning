function [desired_state] = testz(t, qn)

if (t <= 0.0)
    pos     = [0; 0; 0];
else
    pos     = [0; 0; 0.05];
end
vel     = [0; 0; 0];
acc     = [0; 0; 0];
yaw     = 0;
yawdot  = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
