function sdot = hybrid_ptmass_pl_transportationEOM(t, s, controlhandle, trajhandle, quad_params, pl_params)
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - 19 x 1, state vector = [xL, yL, zL, xLd, yLd, zLd,
%                                         xQ, yQ, zQ, xQd, yQd, zQd,
%                                         qw, qx, qy, qz, pQ, qQ, rQ]
%                   ,where [xL, yL, zL] is payload postion
%                          [xLd, yLd, zLd], is payload linear velocity
%                          [xQ, yQ, zQ] is quadrotor position
%                          [xQd, yQd, zQd] is quadrotor velocity
%                          [qw, qx, qy, qz] is quadrotor orientation in quaternion
%                          [pQ, qQ, rQ] is quadrotor angular velocity in its own frame
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% quad_params   - struct, quadrotor parameters you want to pass in
% pl_params     - struct, payload parameters you want to pass in

% OUTPUTS:
% sdot          - 19 x 1, derivative of state vector s

global cable_is_slack;

% if ~pl_params.sim_start
%     cable_is_slack = true;
% end

l = pl_params.cable_length;

% convert state to quad stuct for control
plqd = stateToPtPl(s(1:6));
plqd.qd_pos = s(7:9);
plqd.qd_vel = s(10:12);
plqd.qd_quat = s(13:16);
plqd.qd_omega = s(17:19);
Rot = QuatToRot(s(13:16));
plqd.qd_rot = Rot';
quad_load_rel_pos = plqd.qd_pos-plqd.pos;
quad_load_rel_vel = plqd.qd_vel-plqd.vel;

% Get desired_state
plqd = trajhandle(t, plqd); %GET DES state at time t

% The desired_state is set in the trajectory generator
%Payload
% plqd.pos_des      = desired_state.pos; % I believe this should be of payload now
% plqd.vel_des      = desired_state.vel;
% plqd.acc_des      = desired_state.acc;
% plqd.jerk_des      = desired_state.jrk;
% 
% % Quad desired attitude
% plqd.quad_yaw_des      = desired_state.yaw; %This can remain for Quad
% plqd.quad_yawdot_des   = desired_state.yawdot;

if cable_is_slack % The cable is at slack state
    
    % Compute controller output
    [F,M] = controlhandle(plqd, t, quad_params, pl_params);
    
    % Clamp controller output based on quadrotor's limit
    [F,M] = clamp_thrust_moment(F, M, quad_params);
    
    % Dynamics computation
    sdot = slack_ptmass_payload_quadEOM_readonly(t,plqd,F,M,quad_params);
    
else % The cable is at taut state
    
    % Compute the cable direction
    plqd.xi = -quad_load_rel_pos/l;
    plqd.xidot = -quad_load_rel_vel/l;
    
    % Compute controller output
    [F,M] = controlhandle(plqd, t, quad_params, pl_params);
    
    % Clamp controller output based on quadrotor's limit
    [F,M] = clamp_thrust_moment(F, M, quad_params);
    
    % Dynamics computation
    sdot = taut_ptmass_payload_quadEOM_readonly(t,plqd,F,M,pl_params,quad_params);
    
end

end
