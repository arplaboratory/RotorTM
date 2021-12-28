function [F, M, trpy, drpy] = geometric_controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;
 
%% Parameter Initialization
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;
g = params.grav;
m = params.mass;
phi = qd{qn}.euler(1);
theta = qd{qn}.euler(2);
psi = qd{qn}.euler(3);
e3 = [0;0;1];

%   The rotation matrix in this function is world to body [bRw] you will
%   need to transpose this matrix to get the body to world [wRb] such that
%   [wP] = [wRb] * [bP], where [bP] is a point in the body frame and [wP]
%   is a point in the world frame
Rot_worldtobody = RPYtoRot_ZXY(phi,theta,psi); 

%% Position control 
jerk_des = qd{qn}.jerk_des;
%Position error
ep = qd{qn}.pos_des-qd{qn}.pos;
%Velocity error
ed = qd{qn}.vel_des-qd{qn}.vel;

% Desired acceleration This equation drives the errors of trajectory to zero.
acceleration_des = qd{qn}.acc_des + params.Kp * ep + params.Kd * ed;  

% Thurst f=(kx*ex kv*ex_dot + mge3 +mxdes_ddot)*Re3
Force = m*g*e3  + m*acceleration_des;
F = Force'*Rot_worldtobody'*e3;

%% Attitude Control 
Rot_des = zeros(3,3);
Z_body_in_world = Force/norm(Force);
Rot_des(:,3) = Z_body_in_world;
X_unit = [cos(yaw_des);sin(yaw_des);0];
Y_body_in_world = cross(Z_body_in_world,X_unit);
Y_body_in_world = Y_body_in_world/norm(Y_body_in_world);
Rot_des(:,2) = Y_body_in_world;
X_body_in_world = cross(Y_body_in_world,Z_body_in_world);
Rot_des(:,1) = X_body_in_world;

% Errors of anlges and angular velocities
e_Rot = Rot_des'*Rot_worldtobody' - Rot_worldtobody*Rot_des;
e_angle = vee(e_Rot)/2;

p_des = -(m/F)*(jerk_des - (Z_body_in_world'*jerk_des)*Z_body_in_world)'*Y_body_in_world;
q_des = (m/F)*(jerk_des - (Z_body_in_world'*jerk_des)*Z_body_in_world)'*X_body_in_world;
r_des = yawdot_des*Z_body_in_world(3);
e_omega = qd{qn}.omega - Rot_worldtobody*Rot_des*[p_des q_des r_des]';

% Moment
% Missing the angular acceleration term but in general it is neglectable.
M = - params.Kpe * e_angle - params.Kde * e_omega + cross(qd{qn}.omega,params.I*qd{qn}.omega);

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
