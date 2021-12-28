function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end
%%
%%%%%%%%initialization%%%%%%%%
nodenumber = map.nodenumber;
leftbound = map.boundary(1,1:3);
blockflag = map.blockflag;
resolution = map.resolution;
xy_res = resolution(1);
z_res = resolution(3);
m = map.segment;
mx = m(1);
my = m(2);
mz = m(3);
[row,~] = size(nodenumber);
%describe the discretized coordinates of start position and goal position
cstart = worldtocell(leftbound,resolution,start);
nodestart = celltonumber(m,cstart);
cgoal = worldtocell(leftbound,resolution,goal);
nodegoal = celltonumber(m,cgoal);

visitednodes = zeros(row,1);
value = [];
path = [];
num_expanded = 0;
reachgoal = 0;
minvalue = 0;
node_current = cstart;
node_currentnumber = nodestart;
distance = inf(row,1);%first set all the values of the nodes are infinity
distance(nodestart) = 0;
% countervisit = 0;
% countervisited = 0;
counterneighbour = 0;

square2 = sqrt(2);


%%
%A* algorithm and dijkstra algorithm
if astar
    %%
    % A* algorithm
    while (~reachgoal)
        nx = node_current(1);
        ny = node_current(2);
        nz = node_current(3);
        neighboursize = 6;
        %give the neighbour points stored in a 'neighbours' matrix
        for i = 1:neighboursize
            switch i
                case 1
                    para = nx - 1;
                    if para > 0
                        left = para + (ny-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(left);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 2
                    para = nx+1 ;
                    if para <= mx
                        right = para + (ny-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(right);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 3
                    para = ny - 1;
                    if para > 0
                        back = nx + (ny-2)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(back);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 4
                    para = ny + 1 ;
                    if para <= my
                        front = nx + ny*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(front);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 5
                    para = nz - 1;
                    if para > 0
                        down = nx + (ny-1)*mx + (nz-2)*my*mx;
                        neighbours(i) = nodenumber(down);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 6
                    para = nz + 1 ;
                    if para <= mz
                        up = nx + (ny-1)*mx + (nz)*my*mx;
                        neighbours(i) = nodenumber(up);
                    else
                        neighbours(i) = 0;
                    end
                    %
                    %                 case 7
                    %                     para1 = nx - 1;
                    %                     para2 = ny + 1;
                    %                     if para1 > 0 && para2 <= my
                    %                         leftup = para1 + (para2-1)*mx + (nz-1)*my*mx;
                    %                         neighbours(i) = nodenumber(leftup);
                    %                     else
                    %                         neighbours(i) = 0;
                    %                     end
                    %
                    %                 case 8
                    %                     para1 = nx - 1;
                    %                     para2 = ny - 1;
                    %                     if para1 > 0 && para2 > 0
                    %                         leftback = para1 + (para2-1)*mx + (nz-1)*my*mx;
                    %                         neighbours(i) = nodenumber(leftback);
                    %                     else
                    %                         neighbours(i) = 0;
                    %                     end
                    %
                    %                 case 9
                    %                     para1 = nx + 1;
                    %                     para2 = ny - 1;
                    %                     if para1 <= mx && para2 >0
                    %                         rightback = para1 + (para2-1)*mx + (nz-1)*my*mx;
                    %                         neighbours(i) = nodenumber(rightback);
                    %                     else
                    %                         neighbours(i) = 0;
                    %                     end
                    %
                    %                 case 10
                    %                     para1 = nx + 1;
                    %                     para2 = ny + 1;
                    %                     if para1 <= mx && para2 <= my
                    %                         rightfront = para1 + (para2-1)*mx + (nz-1)*my*mx;
                    %                         neighbours(i) = nodenumber(rightfront);
                    %                     else
                    %                         neighbours(i) = 0;
                    %                     end
                    
            end
        end
        counterneighbour =counterneighbour+1;
        %neighbourcheck(counterneighbour,:) = neighbours;
        %store and compare the distance
        distance_to_neighbours = distance(node_currentnumber)*ones(1,neighboursize) +...
            [xy_res xy_res xy_res xy_res z_res z_res];
        for i = 1:neighboursize
            %meaning the neighbour node is out of boundary
            if neighbours(i) == 0
                continue
            end
            %          if visitednodes(neighbours(i)) == 1
            %              continue
            %          end
            if blockflag(neighbours(i)) == 0 && visitednodes(neighbours(i)) == 0
                %distance_temp = distance(node_currentnumber) + z_res;
                if (distance_to_neighbours(i) < distance(neighbours(i)))
                    distance(neighbours(i)) = distance_to_neighbours(i);
                    previousnode(neighbours(i)) = node_currentnumber;
                    cellnumber_temp = numbertocell(map,neighbours(i));
                    world_temp = celltoworld(leftbound,resolution,cellnumber_temp);
                    eulidician_distance = norm(world_temp-goal);
                    Q(neighbours(i)) = distance(neighbours(i)) + eulidician_distance;
                    value = [Q(neighbours(i)),distance(neighbours(i)),neighbours(i);value];
                    
                end
                
            end
        end
        
        
        visitednodes(node_currentnumber) = 1;
        %lastcurrentnumber = node_currentnumber;
        %blockflag(node_currentnumber) = 1;
        [distance_min, index] = min(value(:,1));
        node_currentnumber = value(index,3);
        
        value(index,:) = [];
        node_current = numbertocell(map,node_currentnumber);
        %            currentcheck(counterneighbour,:) = node_currentnumber;
        num_expanded = num_expanded + 1;
        %            visitednodenumber(num_expanded) = node_currentnumber;
        if node_currentnumber == nodegoal
            reachgoal = 1;
            %previousnode(node_currentnumber) = lastcurrentnumber;
        end
        %A = size(value);
    end
    
    
else
    %%
    %%dijkstra algorithm
    neighboursize = 10;
    while (~reachgoal)
        nx = node_current(1);
        ny = node_current(2);
        nz = node_current(3);
        
        %give the neighbour points stored in a 'neighbours' matrix
        for i = 1:neighboursize
            switch i
                case 1
                    para = nx - 1;
                    if para > 0
                        left = para + (ny-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(left);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 2
                    para = nx+1 ;
                    if para <= mx
                        right = para + (ny-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(right);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 3
                    para = ny - 1;
                    if para > 0
                        back = nx + (ny-2)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(back);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 4
                    para = ny + 1 ;
                    if para <= my
                        front = nx + ny*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(front);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 5
                    para = nz - 1;
                    if para > 0
                        down = nx + (ny-1)*mx + (nz-2)*my*mx;
                        neighbours(i) = nodenumber(down);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 6
                    para = nz + 1 ;
                    if para <= mz
                        up = nx + (ny-1)*mx + (nz)*my*mx;
                        neighbours(i) = nodenumber(up);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 7
                    para1 = nx - 1;
                    para2 = ny + 1;
                    if para1 > 0 && para2 <= my
                        leftup = para1 + (para2-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(leftup);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 8
                    para1 = nx - 1;
                    para2 = ny - 1;
                    if para1 > 0 && para2 > 0
                        leftback = para1 + (para2-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(leftback);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 9
                    para1 = nx + 1;
                    para2 = ny - 1;
                    if para1 <= mx && para2 >0
                        rightback = para1 + (para2-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(rightback);
                    else
                        neighbours(i) = 0;
                    end
                    
                case 10
                    para1 = nx + 1;
                    para2 = ny + 1;
                    if para1 <= mx && para2 <= my
                        rightfront = para1 + (para2-1)*mx + (nz-1)*my*mx;
                        neighbours(i) = nodenumber(rightfront);
                    else
                        neighbours(i) = 0;
                    end
                    
            end
        end
        counterneighbour =counterneighbour+1;
        %neighbourcheck(counterneighbour,:) = [neighbours node_currentnumber 0];
        %store and compare the distance
        distance_to_neighbours = distance(node_currentnumber)*ones(1,neighboursize) +...
            [xy_res xy_res xy_res xy_res z_res z_res square2*xy_res square2*xy_res square2*xy_res square2*xy_res];
        for i = 1:neighboursize
            %meaning the neighbour node is out of boundary
            if neighbours(i) == 0
                continue
            end
            %          if visitednodes(neighbours(i)) == 1
            %              continue
            %          end
            if blockflag(neighbours(i)) == 0 && visitednodes(neighbours(i)) == 0
                
                if (distance_to_neighbours(i) < distance(neighbours(i)))
                    distance(neighbours(i)) = distance_to_neighbours(i);
                    previousnode(neighbours(i)) = node_currentnumber;
                    value = [distance(neighbours(i)),neighbours(i);value];
                end
                
            end
            
        end
        
        visitednodes(node_currentnumber) = 1;
        [distance_min, index] = min(value(:,1));
        node_currentnumber = value(index,2);
        value(index,:) = [];
        node_current = numbertocell(map,node_currentnumber);
        % currentcheck(counterneighbour,:) = node_currentnumber;
        num_expanded = num_expanded + 1;
        visitednodenumber(num_expanded) = node_currentnumber;
        if node_currentnumber == nodegoal
            reachgoal = 1;
            %previousnode(node_currentnumber) = lastcurrentnumber;
        end
        % A = size(value);
        %neighbourcheck(counterneighbour,12) = node_currentnumber;
    end
end
pathlength = length(previousnode);
path(1,:) = goal;
pathnode = nodegoal;
while(pathnode ~= nodestart)
    pathnode = previousnode(pathnode);
    pathnodecell = numbertocell(map,pathnode);
    path = [celltoworld(leftbound,resolution,pathnodecell);path];
end
path = [start;path];

