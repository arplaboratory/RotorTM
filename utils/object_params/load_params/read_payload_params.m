function params = read_payload_params(path,params)
%% read_payload_params: reading physical parameters for the payload
% This function creates a struct with the basic parameters for a payload
% based on the given yaml file.
% 2021 Guanrui Li
%
% INPUTS:
% path          - string, the path to the payload parameter YAML file
% 
% OUTPUTS:
% params        - struct, struct that contains the payload parameters
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, crazyflie

yaml = ReadYaml(path);
field = fieldnames(yaml);

%% Mecahnical Parameters
idx_mass = find(strcmp(field,'mass'));
if ~isempty(idx_mass)
    params.mass = yaml.mass;
else
    error("Missing mass in the config file.")
end

idx_inertia = find(strcmp(field,'inertia'));
if ~isempty(idx_inertia)
    params.payload_type = "Rigid Body";
    params.I = [yaml.inertia.Ixx,   yaml.inertia.Ixy,   yaml.inertia.Ixz;
                yaml.inertia.Iyx,   yaml.inertia.Iyy,   yaml.inertia.Iyz;
                yaml.inertia.Izx,   yaml.inertia.Izy,   yaml.inertia.Izz]; % inertial tensor in m^2 kg
    params.invI = inv(params.I);
else
    params.payload_type = "Point Mass";
    params.I = zeros(3,3);
    params.invI = zeros(3,3);
end

params.grav = 9.81;

%% Visualization
idx_mesh_path = find(strcmp(field,'mesh_path'));
if ~isempty(idx_mesh_path)
    params.mesh_path = yaml.mesh_path;
end

%% Sim Parameters
params.sim_start = false;

end
