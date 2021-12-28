function [v1,v2] = ptmass_inelastic_cable_collision(x1,x2,m1,m2)

obj1_pos = x1(1:3);
obj2_pos = x2(1:3);
obj1_vel = x1(4:6);
obj2_vel = x2(4:6);

cable_direction = (obj2_pos - obj1_pos)/norm(obj2_pos - obj1_pos);
cable_direction_projmat = cable_direction * cable_direction';

v1_proj = cable_direction_projmat * obj1_vel;
v2_proj = cable_direction_projmat * obj2_vel;

v = (m1 * v1_proj + m2 * v2_proj)/(m1+m2);

v1 = v + obj1_vel - v1_proj;
v2 = v + obj2_vel - v2_proj;

end