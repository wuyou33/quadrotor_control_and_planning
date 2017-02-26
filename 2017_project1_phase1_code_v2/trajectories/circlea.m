function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle
% qn is just 1, t < 30
% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

a_z = 0.04;%0.15625; 
a_r = a_z/2.5*2*pi;
T = sqrt(2.5/a_z)*2;
% assume T to be 8s for 1 circle
%  as fast as possible trajectory 
if (t>=0) && (t<T/2)
    theta1 = a_r*t^2/2;
    pos = [5*cos(theta1); 5*sin(theta1) ; a_z*t^2/2];
    vel = [-5*a_r*t*sin(theta1);5*a_r*t*cos(theta1); a_z*t];
    acc = [-5*a_r*(sin(theta1)+a_r*t^2*cos(theta1)); 5*a_r*(cos(theta1)-a_r*t^2*sin(theta1)); a_z];
    
elseif (t>=T/2) && (t<=T)
    theta2 = 2*pi- a_r*(T-t)^2/2;
    pos = [5*cos(theta2); 5*sin(theta2); 2.5-a_z*(T-t)^2/2];
    vel = [-5*a_r*(T-t)*sin(theta2); 5*a_r*(T-t)*cos(theta2);a_z*(T-t)]; 
    acc = [-5*a_r*(-sin(theta2)+a_r*(T-t)^2*cos(theta2)); 5*a_r*(-cos(theta2)-a_r*(T-t)^2*sin(theta2)); -a_z];
else
     pos = [5; 0; 2.5];
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
