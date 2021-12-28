%function [A,b,Aeq,beq] = generate_constraint(path,max_exponent,max_diff,traj_num,max_vel,max_acc)
function [A,b,Aeq,beq] = generate_constraint(path,traj_constant)

%% Initialization
max_exponent = traj_constant.max_exponent;
max_diff = traj_constant.max_diff;
traj_num = size(path,1)-1;
max_vel = traj_constant.max_vel;
max_acc = traj_constant.max_acc;
num_coeff = traj_constant.num_coeff;
dim = traj_constant.dim;

% TODO: To add velocity and acceleration limit to the trajectory
vel_limit = [max_vel max_vel max_vel];
accel_limit = [max_acc max_acc max_acc];
%A = zeros(1,traj_num*dim*num_coeff);
A = [];
Aeq = [];
Aeq_pos = [];
Aeq_vel = [];
Aeq_acc = [];
b = [];
beq = [];
beq_pos = [];
beq_vel = [];
beq_acc = [];

%% vel and acc constraints on the waypoints in the path
%
% if traj_ind == traj_num
%
%     Aeq = [Aeq;poly_coeff_end(2,:) zeros(1,num_coeff)    zeros(1,num_coeff);
%         zeros(1,num_coeff)    poly_coeff_end(2,:) zeros(1,num_coeff);
%         zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_end(2,:);
%         poly_coeff_end(3,:) zeros(1,num_coeff)    zeros(1,num_coeff);
%         zeros(1,num_coeff)    poly_coeff_end(3,:) zeros(1,num_coeff);
%         zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_end(3,:)];
%     beq = [beq;zeros(6,1)];
%
% end

for j = 1:traj_num
    
    pos_constraint_start = path(j,1:3);
    pos_constraint_end   = path(j+1,1:3);
    
    t_start = path(j,4);
    t_end   = path(j+1,4);
    
    %generate polynomial
    poly_coeff_start = generate_poly(max_exponent,max_diff,t_start);
    poly_coeff_end = generate_poly(max_exponent,max_diff,t_end);
    
    %pos constraints on jth trajectory(between the jth waypoint and j+1th waypoint) in the path
    Aeq_pos_j = [blkdiag(poly_coeff_start(1,:),poly_coeff_start(1,:),poly_coeff_start(1,:));
        blkdiag(poly_coeff_end(1,:),poly_coeff_end(1,:),poly_coeff_end(1,:))];
    beq_pos_j = [pos_constraint_start';pos_constraint_end'];
    Aeq_pos((j-1)*dim*2+1:j*dim*2,(j-1)*dim*num_coeff+1:j*dim*num_coeff) = Aeq_pos_j;
    beq_pos((j-1)*dim*2+1:j*dim*2,1) = beq_pos_j;
    
    %vel polynomial terms on the jth waypoint in the path, constructed in
    %the matrix Aeq
    Aeq_vel_j_start = blkdiag(poly_coeff_start(2,:),poly_coeff_start(2,:),poly_coeff_start(2,:));
    
    %acc polynomial terms on the jth waypoint in the path, constructed in
    %the matrix Aeq
    Aeq_acc_j_start = blkdiag(poly_coeff_start(3,:),poly_coeff_start(3,:),poly_coeff_start(3,:));
    
    if j == 1
        
        %vel and acc constraints on the 1st waypoint in the path
        beq_vel_j_start = zeros(dim,1);
        beq_acc_j_start = zeros(dim,1);
        
        Aeq_vel((j-1)*dim+1:j*dim,(j-1)*dim*num_coeff+1:j*dim*num_coeff) = Aeq_vel_j_start;
        Aeq_acc((j-1)*dim+1:j*dim,(j-1)*dim*num_coeff+1:j*dim*num_coeff) = Aeq_acc_j_start;
        beq_vel((j-1)*dim+1:j*dim,1) = beq_vel_j_start;
        beq_acc((j-1)*dim+1:j*dim,1) = beq_acc_j_start;
        
    else
        
        %vel and acc constraints on the jth waypoint
        %for continuity of the j-1th trajectory and the jth trajectory
        beq_vel_j_start = zeros(dim,1);
        beq_acc_j_start = zeros(dim,1);
        
        Aeq_vel((j-1)*dim+1:j*dim,(j-2)*dim*num_coeff+1:j*dim*num_coeff) = [Aeq_vel_j_start -Aeq_vel_j_start];
        Aeq_acc((j-1)*dim+1:j*dim,(j-2)*dim*num_coeff+1:j*dim*num_coeff) = [Aeq_acc_j_start -Aeq_acc_j_start];
        beq_vel((j-1)*dim+1:j*dim,1) = beq_vel_j_start;
        beq_acc((j-1)*dim+1:j*dim,1) = beq_acc_j_start;
        

        
    end
    
    %vel and acc constraints on the end waypoint
    %for stopping at the goal position   
    if j == traj_num
        
        Aeq_vel_j_end = blkdiag(poly_coeff_end(2,:),poly_coeff_end(2,:),poly_coeff_end(2,:));
        Aeq_acc_j_end = blkdiag(poly_coeff_end(3,:),poly_coeff_end(3,:),poly_coeff_end(3,:));
        beq_vel_j_end = zeros(dim,1);
        beq_acc_j_end = zeros(dim,1);
        
        Aeq_vel(j*dim+1:(j+1)*dim,(j-1)*dim*num_coeff+1:j*dim*num_coeff) = Aeq_vel_j_end;
        Aeq_acc(j*dim+1:(j+1)*dim,(j-1)*dim*num_coeff+1:j*dim*num_coeff) = Aeq_acc_j_end;
        beq_vel(j*dim+1:(j+1)*dim,1) = beq_vel_j_end;
        beq_acc(j*dim+1:(j+1)*dim,1) = beq_acc_j_end;
        
    end
    
end

Aeq = [Aeq_pos;Aeq_vel;Aeq_acc];
beq = [beq_pos;beq_vel;beq_acc];

end