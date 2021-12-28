function [pl, qd] = stateToPlQdTeamCable(s,pl_params,quad_params)
%Converts qd struct used in hardware to x vector used in simulation
% x is 1 x 13 vector of state variables [pos vel quat omega]
% qd is a struct including the fields pos, vel, euler, and omega

%% Parameters of the robot team and payload
pl_dim_num = 13;
quad_dim_num = 13;
nquad = pl_params.nquad;

rho_vec_list = pl_params.rho_vec_list;
cable_len_list = pl_params.cable_length;

%% Set up the payload state vector and quadrotor state vectors
pl_state = s(1:pl_dim_num);
qd_state = reshape(s(pl_dim_num + 1 : end),quad_dim_num, nquad);
qd_pos = qd_state(1:3,:);
qd_vel = qd_state(4:6,:);

%% Setup the payload state structure
pl = stateToPl(pl_state);
pl_rot = pl.rot; % The payload rotation matrix w.r.t world frame.
pl_omega_asym = vec2asym(pl.omega);

robot_attach_vector = pl.pos + pl_rot*rho_vec_list - qd_pos;
qd_xi = robot_attach_vector./vecnorm(robot_attach_vector,2,1);
qd_xidot = (pl.vel + pl_rot * pl_omega_asym * rho_vec_list - qd_vel)./cable_len_list;
cable_omg = cross(qd_xi,qd_xidot);
pl.xi = qd_xi;
pl.xidot = qd_xidot;
pl.cable_omg = cable_omg;

%% Setup the quadrotor state structure
for qn = 1:nquad
    
    qd{qn} = stateToQd(qd_state(:,qn));
    qd{qn}.xi = qd_xi(:,qn);
    qd{qn}.xixiT = qd_xi(:,qn)*qd_xi(:,qn)';
    qd{qn}.xidot = qd_xidot(:,qn);
    rho_qn_asym = pl_params.rho_vec_asym_mat(:,3*qn-2:3*qn);
    
    attach_centrifugal_accel = pl_omega_asym * pl_omega_asym * rho_vec_list(:,qn);
    
    qd{qn}.omega_qn = cable_omg(:,qn);
    %qd{qn}.attach_accel = pl_accel + quad_params.grav*[0;0;1] - pl.rot*rho_qn_asym*pl_ang_accel + pl.rot*attach_centrifugal_accel;
    %qd{qn}.attach_accel = attach_accel(:,qn);
    
    quad_params.l = cable_len_list(qn);
    quad_mass_diagonal = quad_params.mass * eye(3);
    
end
