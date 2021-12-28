%function plot_trajectory(traj_num, polynomial_coeff, dim, coefficient, timelist)
function plot_trajectory(traj_constant, polynomial_coeff, coefficient, timelist)
% PLOT_PATH Visualize the trajectory through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

traj_num = traj_constant.total_traj_num;
dim = traj_constant.dim;
pt_num = traj_constant.pt_num;
hold on;

for i = 1:traj_num
    
        if (traj_constant.pt_num == 2) || (mod(i,traj_constant.pt_num-1) == 1)
            t_start = timelist(i);
        end
    
    for t_current = timelist(i):0.01:timelist(i+1)
        
        %time_term = t_current - timelist(i);
        time_term = t_current - t_start;
        
        %time_matrix = [1 time_term time_term^2 time_term^3 time_term^4 time_term^5 time_term^6;
        %    0 1         time_term   time_term^2 time_term^3 time_term^4 time_term^5;
        %    0 0         1           time_term   time_term^2 time_term^3 time_term^4];
        time_matrix = generate_polynomial_matrix(traj_constant.max_exponent,1,time_term);
        %time_sym_matrix = [1 symbol symbol^2 symbol^3 symbol^4 symbol^5 symbol^6;
        %    0 1         symbol   symbol^2 symbol^3 symbol^4 symbol^5;
        %    0 0         1           symbol   symbol^2 symbol^3 symbol^4];
        state = polynomial_coeff(1,:).* time_matrix * coefficient(:,(i-1)*traj_constant.dim+1:i*traj_constant.dim);
        plot3(state(1,1),state(1,2),state(1,3),'g.');
        
    end
end

hold off;

end