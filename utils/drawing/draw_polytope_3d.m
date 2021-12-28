function draw_polytope_3d(A,b,map_boundary)

hold on
vertices = lcon2vert([-eye(3);eye(3);A],[-map_boundary(1:3)';map_boundary(4:6)';b']);

alpha = 0.1;

drawPolyFromVertices(vertices,'b',alpha)

end