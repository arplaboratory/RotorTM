function simplified_path = simplify_path(old_path,map)

basicdata  = map.basicdata;
[rowbasicdata ~] = size(basicdata);
if rowbasicdata >= 2
    block = basicdata(2:rowbasicdata,:);
else
    block = [];
end

counterforfinalpath = 0;

[row ~] = size(old_path);

situation = [];

for i = 1:3
    for j = 1:row-1
        %----------------------------------------------------------
        %If the coordinate is the same as the previous one in x/y/z
        %direction, the corresponding situation is considered as o
        %----------------------------------------------------------
        if old_path(j,i) == old_path(j+1,i)
            situation(j,i) = 0;
        else
            situation(j,i) = 1;
        end
        
    end
end
situation = [0 0 0;situation];
i = 1;
%situation(:,4:6) = old_path;
counter_wave = 0 ;
counter6 = 1 ;
countercheck = 0;

%This loop is designed for eliminating the sawtooth wave in the x-y plane path.
path_no_wave = old_path;
[row1 ~] = size(situation);
while (i ~= (row1-counter_wave))
    if (situation(i,1) ~= situation(i+1,1)) && (situation(i,2) ~= situation(i+1,2)) &&...
            (situation(i,1) == situation(i+2,1)) && (situation(i,2) == situation(i+2,2))
        %condition1: the motion situation is not equal to the
        %            previous one but equal to the latter one
        path_no_wave(i+1,:) = [];
        situation(i+1,:)   = [];
        counter_wave = counter_wave + 1;
    end
    i = i+1;
    %  row = row - counter1;
end

[row2 width] = size(path_no_wave);
situationbetter1 = [];

for i = 1:3
    for j = 1:row2-1
        if path_no_wave(j,i) == path_no_wave(j+1,i)
            situationbetter1(j,i) = 0;
        else
            situationbetter1(j,i) = 1;
        end
        
    end
end
situationbetter1 = [0 0 0;situationbetter1];

k = 1;
while (k ~= row2)
    if situationbetter1(k,1) ~= situationbetter1(k+1,1) ||...
            situationbetter1(k,2) ~= situationbetter1(k+1,2) ||...
            situationbetter1(k,3) ~= situationbetter1(k+1,3)
        counterforfinalpath = counterforfinalpath + 1;
        finalpath1(counterforfinalpath,:) = path_no_wave(k,:);
    end
    k = k+1;
end

finalpath1 = [finalpath1;path_no_wave(row2,:)];
[pathnodenumber ~] = size(finalpath1);
pathcheck = finalpath1;
if (~isempty(block))
    while(counter6 < pathnodenumber-1)
        pointstart = finalpath1(counter6,  :);
        pointend   = finalpath1(counter6+2,:);
        pointx = pointstart(1):(pointend(1)-pointstart(1))/1000:pointend(1);
        pointy = pointstart(2):(pointend(2)-pointstart(2))/1000:pointend(2);
        pointz = pointstart(3):(pointend(3)-pointstart(3))/1000:pointend(3);
        if isempty(pointx)
            points = [pointstart(1)*ones(1001,1) pointy' pointz'];
        elseif isempty(pointy)
            points = [pointx' pointstart(2)*ones(1001,1) pointz'];
        elseif isempty(pointz)
            points = [pointx' pointy' pointstart(3)*ones(1001,1)];
        else
            points = [pointx' pointy' pointz'];
        end
        countercheck = countercheck + 1;
        pointcheck{countercheck,1} = [points(1,:) points(1001,:)];
        C = collide(map,points);
        if isempty(find(C==1))
            finalpath1(counter6+1,:) = [];
            pathnodenumber = pathnodenumber - 1;
        else
            counter6 = counter6+1;
        end
    end
end

simplified_path = finalpath1;

end