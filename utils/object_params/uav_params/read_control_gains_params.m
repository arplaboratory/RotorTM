function params = read_control_gains_params(path,params)
%% read_payload_params: reading controller gains
% This function creates a struct with the basic gains for the UAVs
% based on the given yaml file.
% 2021 Guanrui Li
%
% INPUTS:
% path          - string, the path to the gains parameter YAML file
% 
% OUTPUTS:
% params        - struct, struct that contains the uav parameters

yaml = ReadYaml(path);
field = fieldnames(yaml);

%% Control Gains
idx_pos = find(strcmp(field,'pos'));
if ~isempty(idx_pos)
    params.Kp = [yaml.pos.x  0  0; 
                 0  yaml.pos.y  0; 
                 0  0  yaml.pos.z]; 
end

idx_vel = find(strcmp(field,'vel'));
if ~isempty(idx_vel)
    params.Kd = [yaml.vel.x  0  0; 
                 0  yaml.vel.y  0; 
                 0  0  yaml.vel.z]; 
end

idx_rot = find(strcmp(field,'rot'));
if ~isempty(idx_rot)
    params.Kpe = [yaml.rot.x  0  0;
                 0  yaml.rot.y  0;
                 0  0  yaml.rot.z];
end

idx_ang = find(strcmp(field,'ang'));
if ~isempty(idx_ang)
    params.Kde = [yaml.ang.x  0  0; 
                 0  yaml.ang.y  0; 
                 0  0  yaml.ang.z]; 
end

idx_xi = find(strcmp(field,'xi'));
if ~isempty(idx_xi)
    params.Kxi = [yaml.xi.x  0  0; 
                  0  yaml.xi.y  0; 
                  0  0  yaml.xi.z]; 
end

idx_omg = find(strcmp(field,'omg'));
if ~isempty(idx_omg)
    params.Kw = [yaml.omg.x  0  0; 
                 0  yaml.omg.y  0; 
                 0  0  yaml.omg.z]; 
end

end
