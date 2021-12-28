function [value,isterminal,direction] = tautToSlack(t,x,cable_length)
% slackToTaut function event function for the integration
%
% INPUTS:
% t             - 1 x 1, time
% x             - 19 x 1,
%                 state vector = [xL, yL, zL, xLd, yLd, zLd, 
%                                 qLw, qLx, qLy, qLz, pL, qL, rL, 
%                                 xQ, yQ, zQ, xQd, yQd, zQd,
%                                 qw, qx, qy, qz, pQ, qQ, rQ]
% nquad         - number of quads
% cable_length  - The cable's length

% OUTPUTS:
% value         - (cable-robot distance) - cable_length
% isterminal    - boolean flag representing stop the integration
% direction     - approaching direction of the value 

value = norm(x(1:3) - x(7:9)) - cable_length + 0.001;     % Detect cable-robot distance = 0
isterminal = 1;   % Stop the integration
direction = -1;   % Negative direction only

end