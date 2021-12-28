function draw_ellipsoid_3d(C,d)

hold on
 th = linspace(0,2*pi,30);
 y = [cos(th);sin(th);zeros(size(th))];
 for phi = linspace(0,pi,10)
    T = makehgtform('xrotate', phi);
    R = T(1:3,1:3);
    y = [y, R * y];
 end
 vertices = C*y+d;
 
 alpha = 0.05;
 drawPolyFromVertices(vertices','r',alpha);
 
end