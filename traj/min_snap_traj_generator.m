function [ state_struct ] = min_snap_traj_generator(t_current, state_struct, path, options)
% TRAJECTORY_GENERATOR: Turn a path into a trajectory
%
%,time_list,coefficient_matrix,polynomial_sym
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], path, options) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (for example, dijkstra function)

% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
persistent mapquad
persistent pathall
persistent coefficient
persistent finalpath
persistent timelist
persistent polynomial_coeff
persistent traj_constant

%desired_state = 0.0;
if nargin ~= 2
    %% Initialization

    pathall = path;
    finalpath = path;
    traj_constant = options;

    if traj_constant.pt_num-1 > traj_constant.total_traj_num
        traj_constant.pt_num = traj_constant.total_traj_num + 1;
    end
    traj_constant.traj_num = traj_constant.pt_num -1;
    polynomial_coeff = generate_poly_coeff(traj_constant);
    
    %% Optimization
    
    % Setting optimization settings
    opoptions = optimoptions('quadprog','Display','iter','MaxIter',traj_constant.max_iteration,'TolFun',1e-6,'TolX',1e-6,'TolCon',1e-6);
    % Time allocation(with or without optimizing the time allocation)
    T_seg_c = allocate_time(finalpath,traj_constant.max_vel,traj_constant.max_acc);
    T_seg = T_seg_c;
    %T_seg = optimal_time_seg(finalpath,traj_constant,T_seg_c,cor_constraint,options);
    % Optimization of trajectory
    [coefficient,timelist] = optimize_traj(finalpath,traj_constant,T_seg,traj_constant.cor_constraint,opoptions);
    
    fprintf("The total time of the trajectory is %d s\n", timelist(end))
    %Visualize the trajectory for debugging
    %plot_trajectory(traj_constant, polynomial_coeff, coefficient, timelist);
    
elseif nargin == 2
           
    for i = 1:traj_constant.total_traj_num
        
        if (traj_constant.pt_num == 2) || (mod(i,traj_constant.pt_num-1) == 1)
            t_start = timelist(i);
        end
        
        if t_current >= timelist(i) && t_current < timelist(i+1)
            
            time_term = t_current - t_start;
            %time_term = t_current;
            time_matrix = generate_polynomial_matrix(traj_constant.max_exponent,3,time_term);
            state = polynomial_coeff(1:4,:).* time_matrix * coefficient(:,(i-1)*traj_constant.dim+1:i*traj_constant.dim);
            
        elseif t_current >= timelist(traj_constant.total_traj_num)
            
            state(1,:) = finalpath(traj_constant.total_traj_num+1,:);
            state(2,:) = [0 0 0];
            state(3,:) = [0 0 0];
            state(4,:) = [0 0 0];
            
        end
    end
    
    state_struct.pos_des = state(1,:)';
    state_struct.vel_des = state(2,:)';
    state_struct.acc_des = state(3,:)';
    state_struct.jrk_des = state(4,:)';    
    state_struct.qd_yaw_des = 0;
    state_struct.qd_yawdot_des = 0;
    state_struct.quat_des = [1,0,0,0];
    state_struct.omega_des = [0,0,0];

end





