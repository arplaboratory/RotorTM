function [F, M, trpy, drpy] = hover_controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent gd;
persistent last_t;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;
%position_now = qd{qn}.pos;
%velocity_now = qd{qn}.vel;
%Eulerangle_now = qd{qn}.euler;
%omega_now = qd{qn}.omega;
%position_tra = qd{qn}.pos_des;
%velocity_tra = qd{qn}.vel_des;
%acceleration_tra = qd{qn}.acc_des;

%% Parameter Initialization
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;
g = params.grav;
m = params.mass;
%Gain matrices
Kp_pos = [ 5  0  0; %12
           0  5  0; %12
           0  0  150]; % 150
Kp_att = [ 5  0  0; %12
           0  5  0; %12
           0  0  150]; % 150
Kd_att = [ 5.5  0  0; %5.5
           0  5.5  0;   % 5.5
           0  0  150]; % 150
Ki_att = [0.004  0  0;%0.004
          0  0.004  0;%0.004
          0  0  0.004];%0.004
Kpe = [0.1  0  0; %0.1
       0  0.1  0;%0.1
       0  0  0.2];%0.2
Kde = [0.004  0  0;%0.004
       0  0.004  0;%0.004
       0  0  0.004];%0.004

%% Position control
%Position error
e_pos = qd{qn}.pos_des-qd{qn}.pos;
vel_des = Kp_pos * e_pos;
%Velocity error
e_vel = vel_des-qd{qn}.vel;


%% Hover controller



% Desired acceleration This equation drives the errors of trajectory to zero.
acceleration_des = qd{qn}.acc_des + Kp * ep + Kd * ed;  

%  Desired roll, pitch and yaw
phi_des = (acceleration_des(1)*sin(yaw_des)-acceleration_des(2)*cos(yaw_des))/g;
theta_des = (acceleration_des(1)*cos(yaw_des)+acceleration_des(2)*sin(yaw_des))/g;
psi_des = yaw_des;

% Errors of anlges and angular velocities
e_angle = [phi_des theta_des psi_des]' - qd{qn}.euler;
e_omega = [0 0 yawdot_des]' - qd{qn}.omega;

% Thurst
F = m*g  + m*acceleration_des(3);

% Moment
M = Kpe * e_angle + Kde * e_omega;

% 
gd(icnt,:) = [t, phi_des, qd{qn}.euler(1)];  % for graphing


% You should fill this in


% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end