function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
fid = fopen(filename);
data = textscan(fid,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
[rowdata coldata] = size(data{1,1});
rawdata = [data{1,2} data{1,3} data{1,4} data{1,5} data{1,6} data{1,7} data{1,8} data{1,9} data{1,10}];
blockline = 1;
[n m] = ismember('boundary',data{1,1});
for counterdata = 1:rowdata
    if counterdata == m
        basicdata(1,:) = rawdata(counterdata,:);
    elseif counterdata ~= m
        blockline = blockline + 1;
        basicdata(blockline,:) = rawdata(counterdata,:);
    end 
end
[row column] = size(basicdata);

%the low left corner of the boundary and 
%the upper right corner of the boudary
leftcorner_bound = basicdata(1,1:3);
rightcorner_bound = basicdata(1,4:6);

%the low left corner of the blocks and 
%the upper right corner of the blocks
if (row >= 2) 
leftcorner_block = basicdata(2:row,1:3) - margin;
rightcorner_block = basicdata(2:row,4:6) + margin;

%expand the matrix of the left corner of the boundary into 
%(row-1)*3 matrix. So is the resolution matrix
for counter1 = 2 : row
    leftcorner_blockstart(counter1-1,:) = leftcorner_bound(1,1:3);
    resolution(counter1-1,:) = [xy_res xy_res z_res];
end



%divide x dimension into mx parts
%divide y dimension into my parts
%divide z dimension into mz parts
m = ceil((rightcorner_bound - leftcorner_bound)./[xy_res xy_res z_res]);
mx = m(1);
my = m(2);
mz = m(3);

%Here the each node of the map is given a number following rules:
%1. increasing from left to right. 
%2. 
node(:,1) = 1 : (mx + (my-1)*mx + (mz-1)*mx*my);
flag(:,1) = zeros(mx + (my-1)*mx + (mz-1)*mx*my,1);
%calculate which node the rightcorner and leftcorner of the blocks 
%lied in.
n_upright_block = fix((rightcorner_block - leftcorner_blockstart)./resolution)+1;
n_lowleft_block = fix((leftcorner_block - leftcorner_blockstart)./resolution)+1;

[row1 column1] = size(n_upright_block);
for i = 1:row1
    if n_upright_block(i,1) > mx
        n_upright_block(i,1) = mx;
    end
    if n_upright_block(i,2) > my
        n_upright_block(i,2) = my;
    end
    if n_upright_block(i,3) > mz
        n_upright_block(i,3) = mz;
    end
    for j = 1:column1
        if n_lowleft_block(i,j) <= 0 
            n_lowleft_block(i,j) = 1;
        end 
    end
end
        
%All the nodes that the blocks lie in is given a flag as 1
for i = 1:row-1
    for counter2 = n_lowleft_block(i,1) : 1 : n_upright_block(i,1)
        for counter3 = n_lowleft_block(i,2) : 1 : n_upright_block(i,2)
            for counter4 = n_lowleft_block(i,3) : 1 : n_upright_block(i,3)
                nodeposition = counter2 + (counter3 - 1)*mx+(counter4-1)*mx*my;
                flag(nodeposition,1) = 1;
            end
        end
    end
end

map.nodenumber = node;
map.blockflag = flag;
map.margin = margin;
map.segment = m;
map.boundary = basicdata(1,1:6);
map.block(1:row-1,1:3) = n_lowleft_block;
map.block(1:row-1,4:6) = n_upright_block;
map.block(1:row-1,7:9) = basicdata(2:row,7:9);
map.obstacle_vertices = zeros(3,8,row-1);
for i = 1:row-1
map.obstacle_vertices(:,1,i) = leftcorner_block(i,:)';
map.obstacle_vertices(:,2,i) = [leftcorner_block(i,1:2),rightcorner_block(i,3)]';
map.obstacle_vertices(:,3,i) = [leftcorner_block(i,1),rightcorner_block(i,2),leftcorner_block(i,3)]';
map.obstacle_vertices(:,4,i) = [rightcorner_block(i,1),leftcorner_block(i,2:3)]';
map.obstacle_vertices(:,5,i) = [rightcorner_block(i,1:2),leftcorner_block(i,3)]';
map.obstacle_vertices(:,6,i) = [rightcorner_block(i,1),leftcorner_block(i,2),rightcorner_block(i,3)]';
map.obstacle_vertices(:,7,i) = [leftcorner_block(i,1),rightcorner_block(i,2:3)]';
map.obstacle_vertices(:,8,i) = rightcorner_block(i,:)';
end
map.resolution = [xy_res xy_res z_res];
map.basicdata = basicdata;

else
    
%divide x dimension into mx parts
%divide y dimension into my parts
%divide z dimension into mz parts
m = ceil((rightcorner_bound - leftcorner_bound)./[xy_res xy_res z_res]);
mx = m(1);
my = m(2);
mz = m(3);

%Here the each node of the map is given a number following rules:
%1. increasing from left to right. 
%2. 
node(:,1) = 1 : (mx + (my-1)*mx + (mz-1)*mx*my);
flag(:,1) = zeros(mx + (my-1)*mx + (mz-1)*mx*my,1);

map.nodenumber = node;
map.blockflag = flag;
map.margin = margin;
map.segment = m;
map.boundary = basicdata(1,1:6);
map.block = [];
map.resolution = [xy_res xy_res z_res];
map.basicdata = basicdata;
end

end
