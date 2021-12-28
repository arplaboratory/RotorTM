function [A,b,Aeq,beq] = generate_constraint_toffset(path,max_exponent,max_diff,start_ind,end_ind,traj_num,last_coeff)

persistent last_path;

A = [];
Aeq = [];
b = [];
beq = [];
num_coeff = max_exponent+1;
traj_ind = start_ind;

%last_pos_constraint_start = last_path(1,1:3);
%last_pos_constraint_end   = last_path(2,1:3);
pos_constraint_start = path(1,1:3);
pos_constraint_end   = path(2,1:3);
t_start = path(1,4);
t_end   = path(2,4);

poly_coeff_start = generate_poly_toffset(max_exponent,max_diff,t_start,t_start);
poly_coeff_end = generate_poly_toffset(max_exponent,max_diff,t_end,t_start);
vel_limit = [5 5 5];
accel_limit = [10 10 10];

Aeq = [Aeq;poly_coeff_start(1,:) zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_start(1,:) zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_start(1,:);
    poly_coeff_end(1,:)   zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_end(1,:)   zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_end(1,:)];

beq = [beq;pos_constraint_start';pos_constraint_end'];

vel_accel_limit_A = [poly_coeff_start(2,:) zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_start(2,:) zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_start(2,:);
    poly_coeff_end(2,:)   zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_end(2,:)   zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_end(2,:);
    poly_coeff_start(3,:) zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_start(3,:) zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_start(3,:);
    poly_coeff_end(3,:)   zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_end(3,:)   zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_end(3,:)];

A   = [A;vel_accel_limit_A;-vel_accel_limit_A];

b = [b;vel_limit';vel_limit';accel_limit';accel_limit';vel_limit';vel_limit';accel_limit';accel_limit'];

Aeq = [Aeq;poly_coeff_start(2,:) zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_start(2,:) zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_start(2,:);
    poly_coeff_start(3,:) zeros(1,num_coeff)    zeros(1,num_coeff);
    zeros(1,num_coeff)    poly_coeff_start(3,:) zeros(1,num_coeff);
    zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_start(3,:)];

if traj_ind == 1 

    beq = [beq;zeros(6,1)];
    
else
    
    last_poly_coeff = reshape(last_coeff,max_exponent+1,3);
    last_t_start = last_path(1,4);
    last_t_end   = last_path(2,4);
    time_term = last_t_end - last_t_start;    
    %time_matrix = [1 time_term time_term^2 time_term^3 time_term^4 time_term^5 time_term^6;
    %    0 1         2*time_term   3*time_term^2 4*time_term^3 5*time_term^4 6*time_term^5;
    %    0 0         2           6*time_term   12*time_term^2 20*time_term^3 30*time_term^4];
    time_matrix = [1 time_term time_term^2 time_term^3 time_term^4 time_term^5; 
        0 1         2*time_term   3*time_term^2 4*time_term^3 5*time_term^4; 
        0 0         2           6*time_term   12*time_term^2 20*time_term^3];
    state = time_matrix * last_poly_coeff;
    vel = state(2,:);
    acc = state(3,:);
    beq = [beq;vel';acc'];
    
    
end

if traj_ind == traj_num
    
    Aeq = [Aeq;poly_coeff_end(2,:) zeros(1,num_coeff)    zeros(1,num_coeff);
        zeros(1,num_coeff)    poly_coeff_end(2,:) zeros(1,num_coeff);
        zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_end(2,:);
        poly_coeff_end(3,:) zeros(1,num_coeff)    zeros(1,num_coeff);
        zeros(1,num_coeff)    poly_coeff_end(3,:) zeros(1,num_coeff);
        zeros(1,num_coeff)    zeros(1,num_coeff)    poly_coeff_end(3,:)];
    beq = [beq;zeros(6,1)];
    
end


last_path = path;

end