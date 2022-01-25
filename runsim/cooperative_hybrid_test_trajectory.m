function [xtraj, xtrajdes, ttraj, terminate_cond] = cooperative_hybrid_test_trajectory(start, loadstart, loadstop, vis, trajhandle,pl_params,quad_params)
% TEST_TRAJECTORY simulates the robot from START to STOP following a PATH
% that's been planned for MAP.
% start - a 3d vector or a cell contains multiple 3d vectors
% stop  - a 3d vector or a cell contains multiple 3d vectors
% map   - map generated by your load_map
% path  - n x 3 matrix path planned by your dijkstra algorithm
% vis   - true for displaying visualization

%pl_params = ptpayload();
nquad = pl_params.nquad;

% ros initialization
worldframe = 'simulator';
payloadframe = 'payload';
% tftree = rostf;

%% ROS Visualizaton Setup
cable_pub = rospublisher("/cable/marker","visualization_msgs/Marker","DataFormat","struct");
cable_marker_scale = 0.01*ones(3);
cable_marker_color = [1.0,0.5,0.5,0.5];

cable_marker_msg = init_marker_msg(cable_pub,5,0,worldframe,cable_marker_scale,cable_marker_color);

for i = 1:nquad
    robot_marker_pub{i} = rospublisher("/quadrotor"+num2str(i)+"/marker","visualization_msgs/Marker","DataFormat","struct");
    quadrotor_marker_scale = 0.5*ones(3);
    quadrotor_marker_color = [1.0,0.0,0.0,1.0];
    quadrotor_mesh = quad_params.mesh_path;
    quadrotor_marker_msg{i} = init_marker_msg(robot_marker_pub{i} ,10,0,worldframe,quadrotor_marker_scale,quadrotor_marker_color,quadrotor_mesh);
end

pl_odom_pub = rospublisher("/payload/marker","visualization_msgs/Marker","DataFormat","struct");
payload_marker_scale = ones(3);
payload_marker_color = [1.0,0.745,0.812,0.941];

system_pub = rospublisher("/system/marker","visualization_msgs/MarkerArray","DataFormat","struct");
system_marker = rosmessage(system_pub);

%Controller and trajectory generator handles
if nquad == 1
    controlhandle = @single_payload_geometric_controller;
    dynamicshandle = @(t,s,pl_params) hybrid_ptmass_pl_transportationEOM(t, s, controlhandle, trajhandle, quad_params, pl_params);
    payload_marker_scale = ones(3)*0.1;
    payload_marker_msg = init_marker_msg(pl_odom_pub,2,0,worldframe,payload_marker_scale,payload_marker_color);
else
    controlhandle = @cooperative_suspended_payload_controller;
    dynamicshandle = @(t,s,pl_params) hybrid_cooperative_rigidbody_pl_transportationEOM(t, s, nquad, controlhandle, trajhandle, quad_params, pl_params);
    payload_mesh = pl_params.mesh_path;
    payload_marker_msg = init_marker_msg(pl_odom_pub,10,0,worldframe,payload_marker_scale,payload_marker_color,payload_mesh);
end

path_pub = rospublisher("/payload/path","visualization_msgs/Marker","DataFormat","struct");
path_scale = 0.01*ones(3);
path_color = [1.0,1.0,0.0,0.0];
path_msg = init_marker_msg(path_pub,4,0,worldframe,path_scale,path_color);

des_path_pub = rospublisher("/payload/des_path","visualization_msgs/Marker","DataFormat","struct");
des_path_scale = 0.01*ones(3);
des_path_color = [1.0,0.341,0.0235,0.549];
des_path_msg = init_marker_msg(des_path_pub,4,0,worldframe,des_path_scale,des_path_color);

% Make cell
if ~iscell(start), start = {start}; end

% Make column vector
for qn = 1:nquad
    start{qn} = start{qn}(:);
end

%% **************************** FIGURES *****************************
% Environment figure
if nargin < 5
    vis = true;
end

fprintf('Initializing figures...\n')
if vis
    %h_fig = figure('Name', 'Environment');
else
    %h_fig = figure('Name', 'Environment', 'Visible', 'Off');
end

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
% Maximum time that the quadrotor is allowed to fly
time_tol = 15;          % maximum simulation time
starttime = 0;          % start of simulation in seconds
tstep     = 0.01;       % this determines the time step at which the solution is given
cstep     = 0.05;       % image capture time interval
nstep     = cstep/tstep;
time      = starttime;  % current time
max_iter  = time_tol / cstep;      % max iteration

pl0 = [loadstart(1:3);0;0;0;loadstart(4:7);0;0;0];
x = pl0;
xtraj{1} = [];
ttraj{1} = [];
xtrajdes{1} = [];
for qn = 1:nquad
    % Get start and stop position
    attach = pl_params.rho_vec_list(:,qn);
    cable_len = pl_params.cable_length(qn);
    x0{qn}    = init_state(start{qn}, 0);
    %cable{qn} = init_cable_state(start{qn},pl0,attach, cable_len);% TODO: Init with the quad position
    xtrajdes{qn+1} = [];
    xtraj{qn+1} = [];
    ttraj{qn+1} = [];
    x = [x;x0{qn}];
end


% Maximum position error of the quadrotor at goal
pos_tol  = 0.05; % m
% Maximum speed of the quadrotor at goal
vel_tol  = 0.05; % m/s

% state vector x (13 + 7*nquad + 6*nquad) x 1:
% x     - [xL, yL, zL, xLd, yLd, zLd,
%          [xL, yL, zL, xLd, yLd, zLd, qw, qx, qy, qz, pQuad, qQuad, rQuad]_i, i = 1,...,nquad
quad_state = x0;
pl_dim_num = 13; % rigid body
quad_dim_num = 7;
cable_dim_num = 6;

global cable_is_slack;
global sim_start;
sim_start = false;
cable_is_slack = ~istaut(x(1:3)+QuatToRot(x(7:10))'*pl_params.rho_vec_list,x(13*[1:nquad] + [1;2;3]),pl_params.cable_length);

%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....\n')
for iter = 1:max_iter
    timeint = time:tstep:time+cstep;
    tic;
    % The simulation must first run on the quadrotors
    % Then the simulation simulates the dynamics of the payload
    % If this is the first iteration in the simulation,
    % initialize the plots of the quads
    
    % The integration stops when the distance between robot and payload
    % equals to the cable length
    inelastic_collision_flag = check_inelastic_collision_idx(x,pl_params);
    while any(inelastic_collision_flag)
        fprintf("collide!\n")
        before_collide_inelastic_collision_flag = inelastic_collision_flag;
        x = rigidbody_quad_inelastic_cable_collision(x,pl_params,quad_params,inelastic_collision_flag);
        fprintf("after collide!\n")
        after_collide_inelastic_collision_flag = check_inelastic_collision_idx(x,pl_params);
        if any((after_collide_inelastic_collision_flag - before_collide_inelastic_collision_flag)>0)
            inelastic_collision_flag = after_collide_inelastic_collision_flag | before_collide_inelastic_collision_flag;
        else
            inelastic_collision_flag = after_collide_inelastic_collision_flag;
        end
    end
    
    % State Integration
    slack_condition = cable_is_slack;
    option = odeset('Events',@(t,x)cooperativeGuard(t,x,pl_params,slack_condition));
    [tsave, xsave, fsave] = ode45(@(t,s)dynamicshandle(t,s,pl_params), timeint, x, option); % timeint: 0.05 second / x: initial states /option: check if certain conditions are met
    x = xsave(end, :)';
    
    % Check if the cable is slack
    cable_is_slack = ~istaut(x(1:3)+QuatToRot(x(7:10))'*pl_params.rho_vec_list,x(13*[1:nquad] + [1;2;3]),pl_params.cable_length);
    
    % Save to traj
    for qn = 1:nquad+1        
            xtraj{qn} = [xtraj{qn},xsave(1:end-1,13*qn-12:13*qn)'];
            ttraj{qn} = [ttraj{qn},tsave(1:end-1)'];
            for i = 1:length(tsave)-1
                desired_state = trajhandle(tsave(i), []);
                xtrajdes{qn} = [xtrajdes{qn},[desired_state.pos_des;desired_state.vel_des;desired_state.quat_des';desired_state.omega_des']];
            end
    end
    
    % Update payload state for visualization
    payload_state = x(1:pl_dim_num);
    payload_rot = QuatToRot(payload_state(7:10))';
    
    % Update quad state and cable state for visualization
    quad_state{1} = x(pl_dim_num+1:end);
    cable_point_list = [];
    for qn = 1:nquad
        quad_and_cable_state = x(pl_dim_num + (quad_dim_num + cable_dim_num) * (qn - 1) + 1:pl_dim_num + (quad_dim_num + cable_dim_num) * qn);
        quad_state{qn}(7:13) = quad_and_cable_state(7:13);
        attach_qn = payload_state(1:3) + payload_rot * pl_params.rho_vec_list(:,qn);
        quad_state{qn}(1:3) = quad_and_cable_state(1:3);
        quad_state{qn}(4:6) = payload_state(4:6) + payload_rot * cross(payload_state(11:13), pl_params.rho_vec_list(:,qn)) - pl_params.cable_length(qn) * quad_and_cable_state(quad_dim_num + 4:end);
        cable_point_list = [cable_point_list,attach_qn,quad_and_cable_state(1:3)];
    end
    
    path_msg.Points(iter).X = payload_state(1);
    path_msg.Points(iter).Y = payload_state(2);
    path_msg.Points(iter).Z = payload_state(3);
    path_time = rostime('now');
    path_msg.Header.Stamp.Sec = uint32(path_time.Sec);
    path_msg.Header.Stamp.Nsec = uint32(path_time.Nsec);
    send(path_pub, path_msg);
    des_path_msg.Points(iter).X = desired_state.pos_des(1);
    des_path_msg.Points(iter).Y = desired_state.pos_des(2);
    des_path_msg.Points(iter).Z = desired_state.pos_des(3);
    des_path_msg.Header.Stamp.Sec = uint32(path_time.Sec);
    des_path_msg.Header.Stamp.Nsec = uint32(path_time.Nsec);
    send(des_path_pub, des_path_msg);
    
    % Update quadrotor visualization

    for qn=1:nquad
        system_marker.Markers(qn)= update_marker_msg(quadrotor_marker_msg{qn},quad_state{qn}(1:3),RotToQuat(QuatToRot(quad_state{qn}(7:10))'),qn);
    end
    
    % Update cable visualization
     system_marker.Markers(nquad+1) = update_line_list_msg(cable_marker_msg,cable_point_list,nquad+1);
    
    % Update payload visualization
     system_marker.Markers(nquad+2) = update_marker_msg(payload_marker_msg,payload_state(1:3),RotToQuat(payload_rot),0);

     send(system_pub, system_marker);
    
    % Update simulation time
    time = time + tsave(end) - tsave(1);
    t = toc
    
    % Pause to make real-time
    if (t < cstep)
        %fprintf('t is smaller than cstep\n')
        pause(cstep - t);
    end
    
    % TODO: Check termination criteria
    terminate_cond = terminate_check(payload_state, time, loadstop, pos_tol, vel_tol, time_tol);
    if terminate_cond
        %break
    end
    
end

fprintf('Simulation Finished....\n')

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
% for qn = 1:nquad+1
%     xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
%     ttraj{qn} = ttraj{qn}(1:iter*nstep);
% end

% Plot position for the payload

    %plot_state(load_pos_plot, LP.state_des_hist(1:3,:), LP.time_hist, 'pos', 'des');

end
