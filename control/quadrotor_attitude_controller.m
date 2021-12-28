function M = quadrotor_attitude_controller(qd, params)
% CONTROLLER quadrotor attitude controller
% The current states are:
% qd.rot, qd.omega
% The desired states are:
% qd.rot_des, qd.omega_des
% Using these current and desired states, you have to compute the desired controls
% INPUTS:
% qd     - 1 x 1, Quads' states
% params - struct, quadrotor parameters
%
% OUTPUTS:
% M      - 3 x 1, moments output from controller (only used in simulation)

Rot = qd.rot;
Rot_des = qd.rot_des;
omega_des = qd.omega_des;

% Errors of anlges and angular velocities
e_Rot = Rot_des'*Rot - Rot'*Rot_des;
e_angle = vee(e_Rot)/2;
e_omega = qd.omega - Rot'*Rot_des*omega_des;

% Moment
M = - params.Kpe * e_angle - params.Kde * e_omega + cross(qd.omega,params.I*qd.omega);

end