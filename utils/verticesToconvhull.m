function [X,Y,Z] = verticesToconvhull(c)
% verticesToconvhull: generate convex hull from vertices

k = convhull(c);
X = reshape(c(k',1), size(k'));
Y = reshape(c(k',2), size(k'));
Z = reshape(c(k',3), size(k'));

end