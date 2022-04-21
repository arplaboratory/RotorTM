%% Initialize circle trajectory
radius = 1.0;
period = 6.0;
duration = 6.0;
[~,payload_stop] = circle([], [], payload_start{1}(1:3),radius,period,duration);
trajhandle = @circle;

%% Initialize line trajectory
% line_quintic_traj_generator([], [],map, payload_path);
% trajhandle = @line_quintic_traj_generator;

%% Initialize Minimum kth derivative trajectory
% [pathlength, ~] = size(payload_path);
% traj_constant.max_diff = 6;
% traj_constant.max_exponent = 11;
% traj_constant.max_vel = 2.0;
% traj_constant.max_acc = 4.0;
% traj_constant.dim = 3;
% traj_constant.num_coeff = traj_constant.max_exponent +1;
% traj_constant.total_traj_num = pathlength - 1;
% traj_constant.cor_wid = 0.5;
% traj_constant.nc = 10;
% traj_constant.pt_num = pathlength;
% traj_constant.cor_constraint = false;
% traj_constant.max_iteration = 300;
% min_snap_traj_generator([], [], payload_path, traj_constant);
% trajhandle = @min_snap_traj_generator;

%% Offline generated traj
% offline_traj([],[],"rigid_links_traj.mat")
% trajhandle = @offline_traj;

