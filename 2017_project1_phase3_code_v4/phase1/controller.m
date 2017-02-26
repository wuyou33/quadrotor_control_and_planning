function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
% persistent p r;

% Thurst 
% own test 20+10 %jump test 20+7 %2/7 100+15
% 5th
kp1=20;%20 %18  %15 %20
kd1=10;%10 %7.5 %6  %8
kp2=kp1;
kd2=kd1;
kp3=kp1;
kd3=kd1;

a1 = qd{qn}.acc_des(1)+kd1*(qd{qn}.vel_des(1)-qd{qn}.vel(1))+kp1*(qd{qn}.pos_des(1)-qd{qn}.pos(1));
a2 = qd{qn}.acc_des(2)+kd2*(qd{qn}.vel_des(2)-qd{qn}.vel(2))+kp2*(qd{qn}.pos_des(2)-qd{qn}.pos(2));
a3 = qd{qn}.acc_des(3)+kd3*(qd{qn}.vel_des(3)-qd{qn}.vel(3))+kp3*(qd{qn}.pos_des(3)-qd{qn}.pos(3));
F = params.mass*(params.grav + a3);


% Desired roll, pitch and yaw
psi_des = qd{qn}.yaw_des;
phi_des = (a1*sin(psi_des)-a2*cos(psi_des))/params.grav;
theta_des = (a1*cos(psi_des)+a2*sin(psi_des))/params.grav;
p_des =0;
q_des =0;
r_des = qd{qn}.yawdot_des;

% p = [p; t, phi_des, qd{qn}.euler(1)];
% r = [r; t, theta_des, qd{qn}.euler(2)];


% Moment 
% own test 20+10 % larger xy more accurate
% 5th
kp_phi=10; %10
kd_phi=1; %1
kp_theta=kp_phi;
kd_theta=kd_phi;
kp_psi=kp_phi;
kd_psi=kd_phi;

M    = 100*params.I*[kd_phi*(p_des-qd{qn}.omega(1))+kp_phi*(phi_des-qd{qn}.euler(1));...
                 kd_theta*(q_des-qd{qn}.omega(2))+kp_theta*(theta_des-qd{qn}.euler(2));...
                 kd_psi*(r_des-qd{qn}.omega(3))+kp_psi*(psi_des-qd{qn}.euler(3))]; 
             


% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

% if t > 12
%      figure(2)
%      plot(p(:, 1), p(:, 2), 'b')
%      hold on
%      plot(p(:,1), p(:, 3), 'r');
%      hold off
%      legend('Desired pitch', 'Actual pitch')
%      figure(3)
%      plot(r(:, 1), r(:, 2), 'b')
%      hold on
%      plot(r(:,1), r(:, 3), 'r');
%      hold off
%      legend('Desired roll', 'Actual roll')
% end

end
