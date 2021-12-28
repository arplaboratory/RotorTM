close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map3.txt', 0.1, 1.0, 0.25);
start = {[2.0  3 5.0]}; %map3
stop  = {[18.0  4.0 5.0]};
%start = {[0.1 5.0 1.0]}; %map2
%stop  = {[5.0 10.0 3.0]};
% start = {[1.0  1.0 5.0]};
% stop  = {[11.0 5.0 5.0]};
nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end

disp('Generating Trajectory ...');

syms t 

[state timelist coeff polynomial_sym] = minimum_snap_trajectory_generator([], [], map, path);
poly = reshape(double(subs([polynomial_sym{:,:}],t,1)),[5,7]);
hold on;

for index = 2:10
    t_current = timelist(index);
    time_matrix = [1 t_current t_current^2 t_current^3 t_current^4 t_current^5 t_current^6;
                   0 1         t_current   t_current^2 t_current^3 t_current^4 t_current^5;
                   0 0         1           t_current   t_current^2 t_current^3 t_current^4];
     state1 = poly(1:3,:).* time_matrix * coeff(7*(index-1)-6:7*(index-1),1:3);
     state2 = poly(1:3,:).* time_matrix * coeff(7*index-6:7*index,1:3);
     
     ans{index} = state1 - state2
            
end

% 
% for t = 1:0.01:5
% 
%      desired_state = minimum_snap_trajectory_generator(t, 1);
%      plot3(desired_state.pos(1),desired_state.pos(2),desired_state.pos(3),'b*')
% 
% end