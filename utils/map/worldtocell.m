function A = worldtocell(leftbound,resolution,point)
%lowleftbound = map.boundary(1,1:3);
%resolution = map.resolution;
A = fix((point - leftbound)./resolution)+1;
end


