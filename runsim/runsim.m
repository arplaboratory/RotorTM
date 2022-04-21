close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map0.txt', 0.1, 1.0, 0.25);

%% System State Initialization
%payload_start = {[-0.007743612044263 0.0 0.548299553583291 0.965925826289068 0.0 -0.258819045102521 0.0]'};
%payload_start = {[0.044492843786191 0.0 0.685449553583291 0.965925826289068 0.0 -0.258819045102521 0.0]'};

payload_start = {[0.0 0.0 0.0 1.0 0.0 0.0 0.0]'};
%payload_start = {[0.3 * sin(pi/6) 0 1.0-0.3*cos(pi/6) 1.0 0.0 0.0 0.0]'};
%payload_stop = {[0.0 0.0 0.5 1.0 0.0 0.0 0.0]'};
payload_stop = {[0.0 0.0 0.096899 1.0 0.0 0.0 0.0]'};
%payload_stop = {[2.0 0.0 1.0 1.0 0.0 0.0 0.0]'};
robot_start = {[0.0  0.0 0.5]}; %map3

a = 0.06;
c = 0.0097;

% robot_team_start = {[-0.1540+a -0.267 0.5+c],[0.3083+a 0.0 0.5+c],[-0.1540+a 0.267 0.5+c]}; %3 robots cable mechanism
robot_team_start = {[0.120346902181123	-0.261621500535413	1.46121231042038],...
                    [0.492611221031234	0.0253757808430251	1.51473496260426],...
                    [0.0873862559425843	0.230007948803314	1.48232598564014]};
%robot_team_start = {[0.3048 -0.3048 0.7286],[0.3048 0.3048 0.7286],[-0.3048 0.3048 0.7286],[-0.3048 -0.3048 0.7286],[0.3048 0 0.7286],[-0.3048 0 0.7286]}; %3 robots cable mechanism
%start = {[-0.1540 -0.267 0.5],[0.3083 0.0 0.5],[-0.1540 0.267 0.5]};

% payload_path = [0.0 0.0 0.0;
%     0.25 -0.25 0.125;
%     0.5 -0.25 0.25;
%     0.75 0.25 0.375;
%     1.0 0.5 0.5;
%     1.25 0.25 0.625;
%     1.5 -0.25 0.75;
%     1.75 -0.25 0.875;
%     2.0 0.0 1.0];%;

% payload_path = [0.0 0.0 0.0;
%     0.25 -0.25 0.125;
%     0.5 -0.25 0.25;
%     0.75 0.25 0.375;
%     1.0 0.0 0.5;
%     1.25 -0.25 0.625;
%     1.5 -0.25 0.75;
%     1.75 0.25 0.875;
%     2.0 0.0 1.0];%TODO:: fix the bug here, there is discontinuous path at the end;

% payload_path = [0.0 0.0 0.0;
%     0.5 -0.25 0.25;
%     1.0 0.0 0.5;
%     1.5 -0.25 0.75;
%     2.0 0.0 1.0];

payload_path = {[0.22 0.0 0.5;
                0.22 0.0 0.5]};%;

%% Trajectory Generation Initilization
init_traj_script;

uav_params_path = 'config/uav_params/snapdragonfly.yaml';
payload_params_path = 'config/load_params/fedex_box_payload.yaml';
quad_controller_params_path = 'config/control_params/dragonfly_control_gains.yaml';
payload_controller_params_path = 'config/control_params/triangular_payload_cooperative_cable_gains.yaml';

quad_params = read_uav_params(uav_params_path);
quad_params = read_control_gains_params(quad_controller_params_path,quad_params);

payload_params_path = 'config/load_params/triangular_cardboard_payload.yaml';
mechanism_params_path = 'config/attach_mechanism/3_robots_cable_mechanism.yaml';
pl_params = robot_team_setup(payload_params_path,uav_params_path,mechanism_params_path);
pl_params = read_control_gains_params(payload_controller_params_path,pl_params);

%payload_params_path = 'config/load_params/triangular_cardboard_payload.yaml';
%mechanism_params_path = 'config/attach_mechanism/3_robots_rigid_links_mechanism.yaml';
%payload_controller_params_path = 'config/control_params/triangular_payload_cooperative_rigidlink_gains.yaml';

payload_params_path = 'config/load_params/long_bar_wood_payload.yaml';
payload_controller_params_path = 'config/control_params/triangular_payload_cooperative_rigidlink_gains.yaml';
mechanism_params_path = 'config/attach_mechanism/2_robots_rigid_links_mechanism.yaml';
rigid_pl_params = robot_team_setup(payload_params_path,uav_params_path,mechanism_params_path);%robot_team_setup_old(trianglepayload(),quad_params,"Rigid Link");
rigid_pl_params = read_control_gains_params(payload_controller_params_path,rigid_pl_params);

payload_params_path = 'config/load_params/pointmass_payload.yaml';
mechanism_params_path = 'config/attach_mechanism/ptmass_cable_mechanism.yaml';
payload_controller_params_path = 'config/control_params/pointmass_cable_gains.yaml';
pt_params = robot_team_setup(payload_params_path,uav_params_path,mechanism_params_path);%robot_team_setup_old(trianglepayload(),quad_params,"Rigid Link");
pt_params = read_control_gains_params(payload_controller_params_path,pt_params);

%% Run trajectory
%[data{1}, data_des{1}, time{1}] = hybrid_test_trajectory(robot_start, payload_start{1}, payload_stop{1}, true, trajhandle,pt_params,quad_params); % with visualization
%[data{1}, data_des{1}, time{1}] = cooperative_hybrid_test_trajectory(robot_team_start, payload_start{1}, payload_stop{1}, true, trajhandle,pl_params,quad_params); % with visualization
[data{3}, data_des{3}, time{3}] = rigid_links_test_trajectory(robot_team_start, payload_start{1}, payload_stop{1}, true, trajhandle, rigid_pl_params, quad_params); % with visualization

%% Save data
save("../plot_data/scripts/matlab_scripts/TRO_2022_simulation_rigi_links_" + ...
    "dataset",'data','data_des','time');
