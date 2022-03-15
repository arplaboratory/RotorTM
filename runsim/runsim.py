import numpy as np
close_('all')
clear('all')
addpath(genpath('./'))
## Plan path
print('Planning ...')
map = load_map('maps/map0.txt',0.1,1.0,0.25)
## System State Initialization
#payload_start = {[-0.007743612044263 0.0 0.548299553583291 0.965925826289068 0.0 -0.258819045102521 0.0]'};
#payload_start = {[0.044492843786191 0.0 0.685449553583291 0.965925826289068 0.0 -0.258819045102521 0.0]'};

payload_start = np.array([np.transpose(np.array([0.0,0.0,0.0,1.0,0.0,0.0,0.0]))])
#payload_start = {[0.3 * sin(pi/6) 0 1.0-0.3*cos(pi/6) 1.0 0.0 0.0 0.0]'};
payload_stop = np.array([np.transpose(np.array([0.0,0.0,0.5,1.0,0.0,0.0,0.0]))])
# payload_stop = {[2.0 0.0 1.0 1.0 0.0 0.0 0.0]'};
robot_start = np.array([np.array([0.0,0.0,0.5])])

a = 0.06
c = 0.0097
#robot_team_start = {[-0.1540+a -0.267 1.5+c],[0.3083+a 0.0 1.5+c],[-0.1540+a 0.267 1.5+c]}; #3 robots cable mechanism
robot_team_start = np.array([np.array([0.3048,- 0.3048,0.7286]),np.array([0.3048,0.3048,0.7286]),np.array([- 0.3048,0.3048,0.7286]),np.array([- 0.3048,- 0.3048,0.7286]),np.array([0.3048,0,0.7286]),np.array([- 0.3048,0,0.7286])])

# robot_team_start = {[-0.1540+a -0.267 1.0+c],[0.3083+a 0.0 1.0+c],[-0.1540+a 0.267 1.0+c]};
#start = {[-0.1540 -0.267 0.5],[0.3083 0.0 0.5],[-0.1540 0.267 0.5]};

# payload_path = [0.0 0.0 0.0;
#     0.25 -0.25 0.125;
#     0.5 -0.25 0.25;
#     0.75 0.25 0.375;
#     1.0 0.5 0.5;
#     1.25 0.25 0.625;
#     1.5 -0.25 0.75;
#     1.75 -0.25 0.875;
#     2.0 0.0 1.0];#;

# payload_path = [0.0 0.0 0.0;
#     0.25 -0.25 0.125;
#     0.5 -0.25 0.25;
#     0.75 0.25 0.375;
#     1.0 0.0 0.5;
#     1.25 -0.25 0.625;
#     1.5 -0.25 0.75;
#     1.75 0.25 0.875;
#     2.0 0.0 1.0];#TODO:: fix the bug here, there is discontinuous path at the end;

payload_path = np.array([[0.0,0.0,0.0],[0.5,- 0.5,0.25],[1.0,0.0,0.5],[1.5,- 0.5,0.75],[2.0,0.0,1.0]])
#;
# payload_path = [0.0 0.0 0.5;
#                 0.0 0.0 0.5];#;

#3.0 0.0 5.0;
#1.0 0.5 0.0;
#1.5 -0.5 0.0;
#2.0 0.5 0.0;
#2.5 -0.5 0.0];

## Trajectory Generation Initilization
init_traj_script
uav_params_path = 'config/uav_params/snapdragonfly.yaml'
payload_params_path = 'config/load_params/fedex_box_payload.yaml'
quad_controller_params_path = 'config/control_params/dragonfly_control_gains.yaml'
payload_controller_params_path = 'config/control_params/triangular_payload_cooperative_cable_gains.yaml'
quad_params = read_uav_params(uav_params_path)
quad_params = read_control_gains_params(quad_controller_params_path,quad_params)
mechanism_params_path = 'config/attach_mechanism/6_robots_cable_mechanism.yaml'
pl_params = robot_team_setup(payload_params_path,uav_params_path,mechanism_params_path)
pl_params = read_control_gains_params(payload_controller_params_path,pl_params)
mechanism_params_path = 'config/attach_mechanism/5_robots_rigid_links_mechanism.yaml'
payload_controller_params_path = 'config/control_params/triangular_payload_cooperative_rigidlink_gains.yaml'
rigid_pl_params = robot_team_setup(payload_params_path,uav_params_path,mechanism_params_path)

rigid_pl_params = read_control_gains_params(payload_controller_params_path,rigid_pl_params)
payload_params_path = 'config/load_params/pointmass_payload.yaml'
mechanism_params_path = 'config/attach_mechanism/ptmass_cable_mechanism.yaml'
payload_controller_params_path = 'config/control_params/pointmass_cable_gains.yaml'
pt_params = robot_team_setup(payload_params_path,uav_params_path,mechanism_params_path)

pt_params = read_control_gains_params(payload_controller_params_path,pt_params)
## Run trajectory
data[0],data_des[0],time[0] = hybrid_test_trajectory(robot_start,payload_start[0],payload_stop,True,trajhandle,pt_params,quad_params)

#[data{2}, data_des{2}, time{2}] = cooperative_hybrid_test_trajectory(robot_team_start, payload_start{1}, payload_stop{1}, map, path, true, trajhandle,pl_params,quad_params); # with visualization
#[data{3}, data_des{3}, time{3}] = rigid_links_test_trajectory(robot_team_start, payload_start{1}, payload_stop{1}, map, path, true, trajhandle, rigid_pl_params, quad_params); # with visualization