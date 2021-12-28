function params = read_uav_params(path, params)
%% read_payload_params: reading physical parameters for the payload
% This function creates a struct with the basic parameters for a UAV
% based on the given yaml file.
% 2021 Guanrui Li
%
% INPUTS:
% path          - string, the path to the uav parameter YAML file
% 
% OUTPUTS:
% params        - struct, struct that contains the uav parameters

yaml = ReadYaml(path);

%% Mecahnical Parameters
I = [yaml.inertia.Ixx,   yaml.inertia.Ixy,   yaml.inertia.Ixz; % inertial tensor in m^2 kg
     yaml.inertia.Iyx,   yaml.inertia.Iyy,   yaml.inertia.Iyz;
     yaml.inertia.Izx,   yaml.inertia.Izy,   yaml.inertia.Izz];

params.mass = yaml.mass;
params.I    = I;
params.invI = inv(I);
params.grav = 9.81;

params.arm_length = yaml.arm_length;
params.maxangle = yaml.max_angle * pi/180;

params.maxF = yaml.num_props * yaml.motor_coefficients * yaml.max_rpm^2 * 1e-3 * params.grav;
params.minF = yaml.num_props * yaml.motor_coefficients * yaml.min_rpm^2 * 1e-3 * params.grav;

%% Visualization
params.mesh_path = yaml.mesh_path;

end
