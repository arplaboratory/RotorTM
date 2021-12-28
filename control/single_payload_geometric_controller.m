function [F,M] = single_payload_geometric_controller(ql, t, qd_params, pl_params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent icnt;
persistent g;
persistent e3;

%% Parameter Initialization
if(~pl_params.sim_start)
    icnt = 0;
    g = pl_params.grav;
    e3 = [0;0;1];
end
icnt = icnt + 1;
quad_m = qd_params.mass;
pl_m = pl_params.mass;
l = pl_params.cable_length;


%% State Initialization
quad_load_rel_pos = ql.qd_pos-ql.pos;
quad_load_rel_vel = ql.qd_vel-ql.vel;
quad_load_distance = norm(quad_load_rel_pos);
xi_ = -quad_load_rel_pos/quad_load_distance;
xixiT_ = xi_ * xi_';
xidot_ = -quad_load_rel_vel/quad_load_distance;
xi_asym_ = vec2asym(xi_);
w_ = cross(xi_, xidot_);
Rot_worldtobody = ql.qd_rot';

%% Payload Position control
%jerk_des = ql.jerk_des;
%Position error
ep = ql.pos_des-ql.pos;
%Velocity error
ed = ql.vel_des-ql.vel;

% Desired acceleration This equation drives the errors of trajectory to zero.
acceleration_des = ql.acc_des + g*e3 + pl_params.Kp * ep + pl_params.Kd * ed;

% Desired yaw and yawdot
yaw_des = ql.qd_yaw_des; %This can remain for Quad
yawdot_des = ql.qd_yawdot_des;

%% Cable Direction Control
% Desired cable direction
mu_des_ = (quad_m + pl_m)*acceleration_des + quad_m*l*(xidot_'*xidot_)*xi_; 
xi_des_ = -mu_des_ / norm(mu_des_);
xi_des_dot_ = [0;0;0];% TODO: design q_des_dot_
w_des_ = cross(xi_des_, xi_des_dot_);
w_des_dot_ = [0;0;0];% TODO: design w_des_dot_
mu_ = xixiT_ * mu_des_;

e_xi = cross(xi_des_, xi_);
e_w = w_ + xi_asym_*xi_asym_*w_des_;

Force = mu_ - quad_m*l*cross(xi_, qd_params.Kxi*e_xi + qd_params.Kw*e_w+(xi_'*w_des_)*xidot_ + xi_asym_*xi_asym_*w_des_dot_);
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

% TODO: implement p_des and q_des using differentially flat property
%p_des = -(m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*Y_body_in_world;
%q_des = (m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*X_body_in_world;
p_des = 0.0;
q_des = 0.0;
r_des = yawdot_des*Z_body_in_world(3);
e_omega = ql.qd_omega - Rot_worldtobody*Rot_des*[p_des q_des r_des]';

% Moment
% Missing the angular acceleration term but in general it is neglectable.
M = - qd_params.Kpe * e_angle - qd_params.Kde * e_omega + cross(ql.qd_omega,qd_params.I*ql.qd_omega);

end