function [value,isterminal,direction] = cooperativeGuard(t,x,pl_params,slack_condition)
% slackToTaut function event function for the integration
%
% INPUTS:
% t             - 1 x 1, time
% x             - (13 + 13*nquad) x 1,
%                 state vector = [xL, yL, zL, xLd, yLd, zLd, 
%                                 qLw, qLx, qLy, qLz, pL, qL, rL, 
%                                 [xQ, yQ, zQ, xQd, yQd, zQd]_i, i = 1,...,nquad
%                                 [qw, qx, qy, qz, pQ, qQ, rQ]_i, i = 1,...,nquad
% nquad         - number of quads
% cable_length  - The cable's length

% OUTPUTS:
% value         - (attach points - robot distance) - cable_length
% isterminal    - boolean flag representing stop the integration
% direction     - approaching direction of the value 

% find the idx of the cables that are slack
idx = 1:pl_params.nquad;
slack_cable_idx = idx(slack_condition == 1);
taut_cable_idx = idx(slack_condition == 0);
num_of_slack_cable = length(slack_cable_idx);
num_of_taut_cable = length(taut_cable_idx);

% The rotation matrix of the payload
RotL = QuatToRot(x(7:10))';

% The attach points' positions correspond to the slack cables. 
attach_pts = x(1:3) + RotL * pl_params.rho_vec_list;

% The quadrotor positions correspond to the slack cables. 
slack_quad_pos_idx = 13*slack_cable_idx + [1;2;3];
taut_quad_pos_idx = 13*taut_cable_idx + [1;2;3];

% Set up the condition to terminate the integration.
% Detect cable-robot distance = 0
value = [vecnorm(x(slack_quad_pos_idx) - attach_pts(:,slack_cable_idx),2,1)-pl_params.cable_length(slack_cable_idx), ...
vecnorm(x(taut_quad_pos_idx) - attach_pts(:,taut_cable_idx),2,1)-pl_params.cable_length(taut_cable_idx)+0.0001]';
isterminal = ones(pl_params.nquad,1);   % Stop the integration
direction = [ones(num_of_slack_cable,1); -ones(num_of_taut_cable,1)];   % Positive direction only 

end