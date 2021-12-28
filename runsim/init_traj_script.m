%% Trajectory Initialization scripts

% trajectory_generator([], [], map, path);
% basicdata  = map.basicdata;
% [rowbasicdata ~] = size(basicdata);`
% if rowbasicdata >= 2
%     block = basicdata(2:rowbasicdata,:);
% else
%     block = [];
% end
%finalpath = simplify_path(path{1},block,map);
%finalpath = simplify_path_with_ellipsoid_cor(path{1},ellipsoid_corridor)


%simplify_path_with_ellipsoid_cor(path{1},ellipsoid_corridor)

%% Initialize circle trajectory
radius = 1.0;
period = 6.0;
duration = 6.0;
[~,payload_stop] = circle([], [], payload_start{1}(1:3),radius,period,duration);
trajhandle = @circle;

%% Initialize jump trajectory
% jump([], [],payload_start{1},payload_stop{1});
% trajhandle = @jump;

%% Initialize Minimum kth derivative trajectory
% [pathlength, ~] = size(payload_path);
% traj_constant.max_diff = 6;
% traj_constant.max_exponent = 11;
% traj_constant.max_vel = 3;
% traj_constant.max_acc = 10;
% traj_constant.dim = 3;
% traj_constant.num_coeff = traj_constant.max_exponent +1;
% traj_constant.total_traj_num = pathlength - 1;
% traj_constant.cor_wid = 0.5;
% traj_constant.nc = 10;
% traj_constant.pt_num = pathlength;
% traj_constant.cor_constraint = false;
% traj_constant.max_iteration = 300;
% min_snap_traj_generator([], [], map, payload_path, traj_constant);
% trajhandle = @min_snap_traj_generator;

