clear
% close all
% clc
%% Initialization of System Params
num_of_robots = 3;
rho1_ = [-0.1540;-0.267;0.01125];
rho2_ = [0.3083;0.0;0.0];
%rho2_ = [0.3083;0.0;0.01125];
rho3_ = [-0.1540;0.267;0.01125];
rho_list = [rho1_;rho2_;rho3_];
P = [eye(3),eye(3),eye(3);vec2asym(rho1_),vec2asym(rho2_),vec2asym(rho3_)];
P_bar = null(P);
cable_length = [0.5;0.5;0.5];
robot_id = [6,2,3];

%% Initialization of bags and ROS topics
folder_path = "/home/guanrui/bags/copr-transportation/RAL_submission/move_around_circle_13_visual_payload_odom_visual_cable_all_vio_T_6s_success/";
robotbag = rosbag(folder_path+"payload.bag");
payload_cen_cmd = select(robotbag,'Topic','/payload/cen_pl_cmd');
payload_cen_cmdStructs = readMessages(payload_cen_cmd,'DataFormat','struct');
payload_odom = select(robotbag,'Topic','/payload/odom');
robot_odom = {};
attach_odom = {};
for i = 1:length(robot_id)
    robot_idx = robot_id(i);
    robot_odom{i} = select(robotbag,'Topic',"/dragonfly" + num2str(robot_idx) + "/odom");
    attach_odom{i} = select(robotbag,'Topic',"/dragonfly" + num2str(robot_idx) + "/attach");
end

%% Load robot position
robot_pos = {};
attach_pos = {};
for i = 1:num_of_robots
    robot_pos{i} = odom_2_pos(robot_odom{i});
    attach_pos{i} = odom_2_pos(attach_odom{i});
end

%% Load payload rotation
payload_rotm = odom_2_rotm(payload_odom);

%% Load payload position
payload_pos = odom_2_pos(payload_odom);

%% Load payload central command from bag file
for i = 1:length(payload_cen_cmdStructs)
    if ~isempty(payload_cen_cmdStructs{i}.Mu)
        payload_cen_cmdStructs = payload_cen_cmdStructs(i:end);
        break
    end
end

for i = 1:length(payload_cen_cmdStructs)
    if isempty(payload_cen_cmdStructs{i}.Mu)
        payload_cen_cmdStructs = payload_cen_cmdStructs(1:i-1);
        break
    end
end

mu.mu = [];
for ind = 1:num_of_robots
    mu_x = cellfun(@(m) double(m.Mu(ind).X),payload_cen_cmdStructs);
    mu_y = cellfun(@(m) double(m.Mu(ind).Y),payload_cen_cmdStructs);
    mu_z = cellfun(@(m) double(m.Mu(ind).Z),payload_cen_cmdStructs);
    mu.mu = [mu.mu;mu_x';mu_y';mu_z'];
end
mu.time = cellfun(@(m) double(double(m.Header.Stamp.Sec)+double(m.Header.Stamp.Nsec)*10e-10),payload_cen_cmdStructs);

%% Extract the Vicon data within the command time range
[closestValue,closestIndex] = closest_time(payload_rotm.time,mu.time);
payload_rotm.rotm = payload_rotm.rotm(:,:,closestIndex);
payload_rotm.time = payload_rotm.time(closestIndex,1);
for i = 1:num_of_robots
    attach_position = attach_pos{i};
    [closestValue,closestIndex] = closest_time(attach_position.time,mu.time);
    attach_pos{i}.x = attach_position.x(closestIndex);
    attach_pos{i}.y = attach_position.y(closestIndex);
    attach_pos{i}.z = attach_position.z(closestIndex);
    attach_pos{i}.time = attach_position.time(closestIndex);
end

%%
[~,nullspace_dim] = size(P_bar);
coeff0 = zeros(nullspace_dim,1);
robot_radius = 0.4;
mu_opt_list = [];
for idx = 1:length(mu.time)
    
    % Initialization
    payload_rotation_matrix = payload_rotm.rotm(:,:,idx);
    cable_tensions = mu.mu(:,idx);
    rot_diag = blkdiag(payload_rotation_matrix,payload_rotation_matrix,payload_rotation_matrix);
    P_bar_world = rot_diag * P_bar;
    rho_vec_world = rot_diag * rho_list;
    
    % Build up constraints
    constraint_idx = 0;
    %     for i = 1:num_of_robots
    %         for j = i+1:num_of_robots
    %             constraint_idx = constraint_idx + 1;
    %             constraint_H = zeros(3*num_of_robots,3*num_of_robots);
    %             constraint_H(3*i-2:3*i,3*i-2:3*i) = eye(3);
    %             constraint_H(3*j-2:3*j,3*j-2:3*j) = eye(3);
    %             constraint_H(3*i-2:3*i,3*j-2:3*j) = -2*eye(3);
    %             H{constraint_idx} = -constraint_H;
    %
    %             constraint_k = zeros(1,3*num_of_robots);
    %             rho_error = rho_vec_world(3*i-2:3*i) - rho_vec_world(3*j-2:3*j);
    %             constraint_k(3*i-2:3*i) = 2 * rho_error';
    %             constraint_k(3*j-2:3*j) = -2 * rho_error';
    %             k{constraint_idx} = -constraint_k;
    %
    %             d{constraint_idx} = 4*robot_radius^2;
    %         end
    %     end
    
    % Optimization
    tic
    fun = @(x)quadobj(x,P_bar_world'*P_bar_world,P_bar_world'*cable_tensions,0);
    nonlconstr = @(x)cable_distribution_constr(x,rho_vec_world,cable_length,P_bar_world,cable_tensions);
    [coeff_opt,fval,eflag,output,lambda] = fmincon(fun,coeff0,...
        [],[],[],[],[],[],nonlconstr);
    time = toc
    
    %coeff_opt = quadprog(P_bar_world'*P_bar_world,cable_tensions'*P_bar_world);
    mu_opt = cable_tensions + P_bar_world * coeff_opt;
    mu_opt_list = [mu_opt_list,mu_opt];
    
    attach_x = [attach_pos{1}.x(idx),attach_pos{2}.x(idx),attach_pos{3}.x(idx),attach_pos{1}.x(idx)];
    attach_y = [attach_pos{1}.y(idx),attach_pos{2}.y(idx),attach_pos{3}.y(idx),attach_pos{1}.y(idx)];
    attach_z = [attach_pos{1}.z(idx),attach_pos{2}.z(idx),attach_pos{3}.z(idx),attach_pos{1}.z(idx)];
    clf
    line(attach_x,attach_y,attach_z);
    for i = 1:num_of_robots
        mu_norm = norm(mu.mu(3*i-2:3*i,idx));
        mu_opt_norm = norm(mu_opt(3*i-2:3*i));
        tension_x = [attach_pos{i}.x(idx),attach_pos{i}.x(idx)+cable_length(i)*mu.mu(3*i-2,idx)/mu_norm];
        tension_y = [attach_pos{i}.y(idx),attach_pos{i}.y(idx)+cable_length(i)*mu.mu(3*i-1,idx)/mu_norm];
        tension_z = [attach_pos{i}.z(idx),attach_pos{i}.z(idx)+cable_length(i)*mu.mu(3*i,idx)/mu_norm];
        tension_new_x = [attach_pos{i}.x(idx),attach_pos{i}.x(idx)+cable_length(i)*mu_opt(3*i-2)/mu_opt_norm];
        tension_new_y = [attach_pos{i}.y(idx),attach_pos{i}.y(idx)+cable_length(i)*mu_opt(3*i-1)/mu_opt_norm];
        tension_new_z = [attach_pos{i}.z(idx),attach_pos{i}.z(idx)+cable_length(i)*mu_opt(3*i)/mu_opt_norm];
        line(tension_x,tension_y,tension_z,'Color','r');
        line(tension_new_x,tension_new_y,tension_new_z,'Color','g')
    end
    view(3);
    zlim([0,1]);
    xlim([-2,2]);
    ylim([-2,2]);
    pause(0.0001)
end














