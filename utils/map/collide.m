function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

[rowbasic colbasic] = size(map.basicdata);
[row1 column1] = size(points);
if rowbasic == 1
    C = zeros(row1,1);
else
data = map.basicdata;
margin = map.margin;
[row column] = size(data);
uprightcorner_bound = map.basicdata(1,4:6);
lowleftcorner_bound = map.basicdata(1,1:3);
 uprightcorner_block = map.basicdata(2:row,4:6)+margin;
 lowleftcorner_block = map.basicdata(2:row,1:3)-margin;
%blockflag = map.blockflag;
resolution0 = map.resolution;

%[row1 column1] = size(points);
[row2 column2] = size(uprightcorner_block);

for i = 1:row1
    leftcorner_boundstart(i,:) = lowleftcorner_bound(1,:);
    resolution(i,:) = resolution0;
end


n_points = points;
%n_points = floor((points - leftcorner_boundstart)./resolution)+1; 
% [rowpt colpt] = size(n_points);
% for i = 1:rowpt
%     for j = 1:colpt
%        if n_points(i,j) == 0
%            n_points(i,j) = 1;
%        end 
%     end
% end
% 
% m_bound = map.segment;
% mx = m_bound(1);
% my = m_bound(2);
% mz = m_bound(3);

[rowbk colbk] = size(uprightcorner_block);
% for counter = 1:2
%     switch counter
%         case 1
            for i = 1:rowbk
                for j = 1:colbk
                    if uprightcorner_block(i,j) > uprightcorner_bound(j)
                       uprightcorner_block(i,j) = uprightcorner_bound(j);
                    end
                end
            end
            
       % case 2 
            for i = 1:rowbk
                for j = 1:colbk
                    if lowleftcorner_block(i,j) < lowleftcorner_bound(j)
                       lowleftcorner_block(i,j) = lowleftcorner_bound(j);
                    end
                end
             end
%     end
% end

for i = 1:row1
    for j = 1:row2
    if n_points(i,1)<=uprightcorner_block(j,1)&&...
       n_points(i,1)>=lowleftcorner_block(j,1)&&...
       n_points(i,2)<=uprightcorner_block(j,2)&&...
       n_points(i,2)>=lowleftcorner_block(j,2)&&...
       n_points(i,3)<=uprightcorner_block(j,3)&&...
       n_points(i,3)>=lowleftcorner_block(j,3)||...
       n_points(i,1)<lowleftcorner_bound(1)||...
       n_points(i,1)>uprightcorner_bound(1)||...
       n_points(i,2)<lowleftcorner_bound(2)||...
       n_points(i,2)>uprightcorner_bound(2)||...
       n_points(i,3)<lowleftcorner_bound(3)||...
       n_points(i,3)>uprightcorner_bound(3)%||...
%        n_points(i,1)<=0||n_points(i,1)>mx||...
%        n_points(i,2)<=0||n_points(i,2)>my||...
%        n_points(i,3)<=0||n_points(i,3)>mz...
       
     C(i,1) = 1;
     break
    else 
     C(i,1) = 0;
    end
    end

end

end

% for i = 1:row1
%     for j = 1:row2
%     if points(i,1)<=uprightcorner_block(j,1)&&...
%        points(i,1)>=lowleftcorner_block(j,1)&&...
%        points(i,2)<=uprightcorner_block(j,2)&&...
%        points(i,2)>=lowleftcorner_block(j,2)&&...
%        points(i,3)<=uprightcorner_block(j,3)&&...
%        points(i,3)>=lowleftcorner_block(j,3)
%      C(i,1) = 1;
%      break
%     else 
%      C(i,1) = 0;
%     end
%     end
% 
% end
