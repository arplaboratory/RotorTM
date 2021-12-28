function [map,path] = LoadMapAndPlanning(index)
%% Load Map
disp("Loading Map ..."+num2str(index));
map = load_map("maps/map"+num2str(index)+".txt", 0.1, 1.0, 0.25);

switch index
    case 1
        start = {[2.0  -2.0 1.0]};%map1
        stop  = {[8.0 18.0 2.5]};
    case 2
        start = {[0.1 5.0 1.0]}; %map2
        stop  = {[5.0 10.0 3.0]};
    case 3
        start = {[2.0  3 5.0]}; %map3
        stop  = {[18.0  4.0 5.0]};
end

%% Plan Path
disp('Planning ...');
path{1} = dijkstra(map, start{1}, stop{1}, true);

end

