function A = celltoworld(leftbound,resolution,point)
%lowleftbound = map.boundary(1,1:3);
%resolution = map.resolution;
nx = point(1);
ny = point(2);
nz = point(3);
xy_res = resolution(1);
z_res = resolution(3);
A(1) = xy_res/2 + (nx-1)*xy_res+leftbound(1);
A(2) = xy_res/2 + (ny-1)*xy_res+leftbound(2);
A(3) = z_res/2 + (nz-1)*z_res+leftbound(3);
end


