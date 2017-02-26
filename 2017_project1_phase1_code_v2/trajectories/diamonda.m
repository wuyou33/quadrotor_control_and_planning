function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

a = sqrt(2)/4; %sqrt(2)
T = sqrt(sqrt(2)/a)*2*4;
a_x = 1/(T/4)^2;
% assume T to be 16s for 1 circle
%  as fast as possible trajectory 
if (t>=0) && (t<T/8)
pos = [a_x*t^2/2; a*t^2/2; a*t^2/2];
vel = [a_x*t; a*t; a*t];
acc = [a_x; a; a];

elseif (t>=T/8) && (t<T/4)
pos = [1/4-a_x*(T/4-t)^2/2; sqrt(2)-a*(T/4-t)^2/2; sqrt(2)-a*(T/4-t)^2/2];
vel = [a_x*(T/4-t); a*(T/4-t); a*(T/4-t)];
acc = [-a_x; -a; -a];

elseif (t>T/4) && (t<T*3/8)
pos = [1/4+a_x*(t-T/4)^2/2; sqrt(2)-a*(t-T/4)^2/2; sqrt(2)+a*(t-T/4)^2/2];
vel = [a_x*(t-T/4); -a*(t-T/4); a*(t-T/4)];
acc = [a_x; -a; a];

elseif (t>=T*3/8) && (t<T/2)
pos = [1/2-a_x*(T/2-t)^2/2; a*(T/2-t)^2/2; 2*sqrt(2)-a*(T/2-t)^2/2];
vel = [a_x*(T/2-t); -a*(T/2-t); a*(T/2-t)];
acc = [-a_x; a; -a];

elseif (t>=T/2) && (t<T*5/8)
pos = [1/2+a_x*(t-T/2)^2/2; -a*(t-T/2)^2/2; 2*sqrt(2)-a*(t-T/2)^2/2];
vel = [a_x*(t-T/2); -a*(t-T/2); -a*(t-T/2)];
acc = [a_x; -a; -a];

elseif (t>=T*5/8) && (t<T*3/4)
pos = [3/4-a_x*(T*3/4-t)^2/2; -sqrt(2)+a*(T*3/4-t)^2/2; sqrt(2)+a*(T*3/4-t)^2/2];
vel = [a_x*(T*3/4-t); -a*(T*3/4-t); -a*(T*3/4-t)];
acc = [-a_x; a; a];

elseif (t>=T*3/4) && (t<T*7/8)
pos = [3/4+a_x*(t-T*3/4)^2/2; -sqrt(2)+a*(t-T*3/4)^2/2; sqrt(2)-a*(t-T*3/4)^2/2];
vel = [a_x*(t-T*3/4); a*(t-T*3/4); -a*(t-T*3/4)];
acc = [a_x; a; -a ];

elseif (t>=T*7/8) && (t<T)
pos = [1-a_x*(T-t)^2/2; -a*(T-t)^2/2; a*(T-t)^2/2];
vel = [a_x*(T-t); a*(T-t); -a*(T-t)];
acc = [-a_x; -a; a];

else
     pos = [1; 0; 0];
     vel = [0; 0; 0];
     acc = [0; 0; 0];
end
%leave as 0
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
