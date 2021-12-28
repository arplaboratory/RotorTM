function plot_path(map, path, coefficient, timelist)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

configuration = map.basicdata;
[len ~] = size(configuration);
hold on;

if nargin == 2
for i = 1 : len
    
    center_x = (configuration(i,1) + configuration(i,4))/2;
    center_y = (configuration(i,2) + configuration(i,5))/2;
    center_z = (configuration(i,3) + configuration(i,6))/2;
    
    x_res =  configuration(i,4) - configuration(i,1);
    y_res =  configuration(i,5) - configuration(i,2);
    z_res =  configuration(i,6) - configuration(i,3);
    
    x_down = center_x - x_res/2;
    x_up = center_x + x_res/2;
    y_down = center_y - y_res/2;
    y_up = center_y + y_res/2;
    z_down = center_z - z_res/2;
    z_up = center_z + z_res/2;
    
    X = [x_down x_down x_down x_down x_down x_up;
        x_up   x_up   x_up   x_up   x_down x_up;
        x_up   x_up   x_up   x_up   x_down x_up;
        x_down x_down x_down x_down x_down x_up];
    Y = [y_down y_down y_up   y_down y_down y_down;
        y_down y_down y_up   y_down y_up   y_up;
        y_down y_up   y_up   y_up   y_up   y_up;
        y_down y_up   y_up   y_up   y_down y_down];
    Z = [z_down z_down z_down z_up   z_down z_down;
        z_down z_down z_down z_up   z_down z_down;
        z_up   z_down z_up   z_up   z_up   z_up;
        z_up   z_down z_up   z_up   z_up   z_up];
    
    if i == 1
        Color = 'w';
        alpha = 0.1;
        
    else
        Color = 'k';
        alpha = 0.5;
    end
        
    fill3(X,Y,Z,Color,'FaceAlpha',alpha)

end

if ~isempty(path)
plot3(path(:,1),path(:,2),path(:,3),'b-')
hold off;
end
xlabel('x')
ylabel('y')
zlabel('z')

elseif  nargin == 4
    

    
end 
end