function plot_obstacle_vertices(map)
%PLOT_OBSTACLE_VERTICES Summary of this function goes here
%   Detailed explanation goes here
ob_vert = map.obstacle_vertices;
[~,~,number_obstacles] = size(ob_vert);
% hold on
% for i = 1:8
% plot3(ob_vert(i*7,1),ob_vert(i*7,2),ob_vert(i*7,3),'*')
% end
for i = 1:number_obstacles
    hold on
    plot3(ob_vert(1,:,i),ob_vert(2,:,i),ob_vert(3,:,i),'*')
end
end

