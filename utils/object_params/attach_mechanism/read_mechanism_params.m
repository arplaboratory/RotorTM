function params = read_mechanism_params(path,params)
%% read_payload_params: reading mechanism parameters for the payload
% This function creates a struct with the basic parameters for the attach
% mechanism based on the given yaml file.
% 2021 Guanrui Li
%
% INPUTS:
% path          - string, the path to the mechanism parameters YAML file
% 
% OUTPUTS:
% params        - struct, struct that contains the mechanism parameters

yaml = ReadYaml(path);

%% Attachment Type
params.type = yaml.type;

%% Specific Parameters for different mechanism
if params.type == "Cable"
    params.cable_length = cell2mat(yaml.cable_length);
elseif params.type == "Rigid Link"
    params.yaw_list = cell2mat(yaml.yaw_list);
else
    error("Invalid attach mechanism");
end

%% Set up the total number of robots
params.nquad = yaml.num_of_robots;

%% Attach Position on the payload
rho_vec_list = [];
for i = 1:params.nquad
    rho = [yaml.rho{i}.x;yaml.rho{i}.y;yaml.rho{i}.z];
    rho_vec_list = [rho_vec_list,rho];
end
params.rho_vec_list = rho_vec_list;




