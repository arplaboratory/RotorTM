function [ load ] = rigidbody_polygon_pos(pos, rot, vertices)
%QUAD_POS Calculates coordinates of quadrotor's position in world frame
% pos       3x1 position vector [x; y; z];
% rot       3x3 body-to-world rotation matrix
% L         1x1 length of the quad

% wRb   = RPYtoRot_ZXY(euler(1), euler(2), euler(3))';
wHb   = [rot pos(:); 0 0 0 1]; % homogeneous transformation from body to world

loadBodyFrame  = [vertices, ones(length(vertices),1)]';%[L 0 0 1; 0 L 0 1; -L 0 0 1; 0 -L 0 1; 0 0 0 1; 0 0 H 1]';
loadWorldFrame = wHb * loadBodyFrame;
load           = loadWorldFrame(1:3, :);

end
