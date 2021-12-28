function [ s ] = init_cable_state(quad,load,attach,len)
%INIT_STATE Initialize 13 x 1 state vector

load_rot = QuatToRot(load(7:10))';
xi = load(1:3) + load_rot * attach - quad;
dist = norm(xi);

if abs(dist-len) > 1e-04
    warning("The distance between the init quad position and the attach is longer than the taut cable len");
    warning("Going to ignore the quad init position and reinitialize it based on the attach point position and the tau cable length assumption");
end

xi_norm = xi/dist;
s    = zeros(6,1);
s(1) = xi_norm(1);%xi_x
s(2) = xi_norm(2);%xi_y
s(3) = xi_norm(3);%xi_z
s(4) = 0;        %xidot_x
s(5) = 0;        %xidot_y
s(6) = 0;        %xidot_z

end