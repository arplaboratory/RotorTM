function [hdl1,hdl2] = drawPolyFromVertices(c,color,alpha)
% drawPolyFromVertices: Draw polygon from vertices
% INPUT:
% c          - (NumOfVertices x Dimension) The matrix contain the coordinates of all vertices
% color      - color used for the polygon
% alpha      - define transparency of the surfaces of polygons
% OUTPUT:
% hdl1       - surface plot handle
% hdl2       - edge plot handle

%[X,Y,Z] = verticesToconvhull(c);
X = c(:,1);
Y = c(:,2);
Z = c(:,3);
[k,~] = convhull(c);
hdl1 = trisurf(k,X,Y,Z,'FaceColor',color,'FaceAlpha',alpha,'EdgeColor','k');
%hdl2 = plot3(X,Y,Z,'k');
%plot3(X,Y,Z,color)
%fill3(X,Y,Z,color, 'FaceAlpha', 0.5)

end