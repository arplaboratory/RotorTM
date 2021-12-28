%function [collided_pl_vel,collided_pl_omg,collided_robot_vel] = rigidbody_quad_inelastic_cable_collision(xL,xQs,pl_params,quad_params,slack_idx)
function x = rigidbody_quad_inelastic_cable_collision(x,pl_params,quad_params,collision_idx)
 
%% State Assignment
xL = x(1:13);
xQs = x(14:end);
nquad = pl_params.nquad;
%collision_idx = ~slack_idx;
ntaut_quad = length(find(collision_idx == 1));
IL = pl_params.I;
mL = pl_params.mass;
rho_vec_asym_mat = pl_params.rho_vec_asym_mat(:,repelem(collision_idx,3));
rho_vec_list = pl_params.rho_vec_list(:,collision_idx);

pl_rot = QuatToRot(xL(7:10))';
pl_omg = xL(11:13);
pl_vel = xL(4:6);
attach_pos = xL(1:3) + pl_rot * rho_vec_list;
robot_state = reshape(xQs,[13,nquad]);
robot_pos = robot_state(1:3,collision_idx);
robot_vel = robot_state(4:6,collision_idx);

%% State Computation
xi = (attach_pos - robot_pos)./vecnorm(attach_pos - robot_pos,2,1);
robot_vel_xi_proj = sum(xi.*robot_vel,1); 
xixiT_robot_vel = xi.*robot_vel_xi_proj;
xi_vert_robot_vel = robot_vel - xixiT_robot_vel;

hatrho_rotL_xi = -(pl_rot*rho_vec_asym_mat)'*xi;
hatrho_rotL_xi = hatrho_rotL_xi((3*ntaut_quad+3)*[0:ntaut_quad-1]+[1:3]');


%% Calculation For Collision
G1 = [xi;-hatrho_rotL_xi];
G2 = [xi;hatrho_rotL_xi]';
ML = [mL*eye(3),zeros(3,3);zeros(3,3),IL];
MQ = quad_params.mass*eye(ntaut_quad);
M = ML + G1*MQ*G2;
b = [mL*pl_vel+xi*MQ*robot_vel_xi_proj';
     - hatrho_rotL_xi * MQ * robot_vel_xi_proj' + IL * pl_omg];

collided_pl_vel_omg = M\b;
collided_pl_vel = collided_pl_vel_omg(1:3);
collided_pl_omg = collided_pl_vel_omg(4:6);

collided_robot_vel_proj = xi.*sum(xi.*(collided_pl_vel+pl_rot*vec2asym(collided_pl_omg)*rho_vec_list),1);
collided_robot_vel = xi_vert_robot_vel + collided_robot_vel_proj;

%% Return States
x(4:6) = collided_pl_vel;
x(11:13) = collided_pl_omg;
counter = 0;
for nq = 1:nquad
    if collision_idx(nq)
        counter = counter + 1;
        x(13*nq + [4;5;6]) = collided_robot_vel(:,counter);
    end
end

    
% nquad = pl_params.nquad;
% IL = pl_params.I;
% mL = pl_params.mass;
% rho_vec_asym_mat = pl_params.rho_vec_asym_mat;
% rho_vec_list = pl_params.rho_vec_list;
% 
% pl_rot = QuatToRot(xL(7:10));
% pl_omg = xL(11:13);
% pl_vel = xL(4:6);
% robot_state = reshape(xQs,[13,nquad]);
% robot_pos = robot_state(1:3,:);
% robot_vel = robot_state(4:6,:);
% attach_pos = xL(1:3) + pl_rot * rho_vec_list;
% 
% %% State Computation
% xi = (attach_pos - robot_pos)./vecnorm(attach_pos - robot_pos,2,1);
% robot_vel_xi_proj = sum(xi.*robot_vel,1); 
% xixiT_robot_vel = xi.*robot_vel_xi_proj;
% xi_vert_robot_vel = robot_vel - xixiT_robot_vel;
% 
% hatrho_rotL_xi = -(pl_rot*rho_vec_asym_mat)'*xi;
% hatrho_rotL_xi = hatrho_rotL_xi((3*nquad+3)*[0:nquad-1]+[1:3]');
% 
% 
% %% Calculation For Collision
% G1 = [xi;-hatrho_rotL_xi];
% G2 = [xi;hatrho_rotL_xi]';
% ML = [mL*eye(3),zeros(3,3);zeros(3,3),IL];
% MQ = quad_params.mass*eye(nquad);
% M = ML + G1*MQ*G2;
% b = [mL*pl_vel+MQ*xi*robot_vel_xi_proj';
%      - MQ * hatrho_rotL_xi * robot_vel_xi_proj' + IL * pl_omg];
% 
% collided_pl_vel_omg = M\b;
% collided_pl_vel = collided_pl_vel_omg(1:3);
% collided_pl_omg = collided_pl_vel_omg(4:6);
% 
% collided_robot_vel_proj = xi.*sum(xi.*(collided_pl_vel+pl_rot*vec2asym(collided_pl_omg)*pl_params.rho_vec_list),1);
% collided_robot_vel = xi_vert_robot_vel + collided_robot_vel_proj;

vk_cable_proj = zeros(3,1);
vk_rho_cable_proj = zeros(3,1);
rho_cable_proj = zeros(3,3);
cable_rho_proj = zeros(3,3);
% fprintf("The collided payload velocity is %f,%f,%f\n", collided_pl_vel);
% fprintf("The collided payload angular velocity is %f,%f,%f\n", collided_pl_omg);

for n = 1:ntaut_quad
    %fprintf("Calculating robot %i\n ",n)
    vk_cable_proj = xi(:,n)*xi(:,n)'*collided_robot_vel(:,n);
    %fprintf("The collided robot velocity is %f,%f,%f\n", vk_cable_proj);
    vk_rho_cable_proj = xi(:,n)*xi(:,n)'*(collided_pl_vel + pl_rot*cross(collided_pl_omg,rho_vec_list(:,n)));
    %fprintf("The collided attach velocity is %f,%f,%f\n", vk_rho_cable_proj);
end


end