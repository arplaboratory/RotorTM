function [y,yeq,grady,gradyeq] = cable_distribution_constr(x,rho_mat,cable_length,Pbar,mu_min_norm,robot_radius)

num_of_robots = length(cable_length);
constraint_idx = 0;

for i = 1:num_of_robots
    for j = i+1:num_of_robots
        
        constraint_idx = constraint_idx + 1;
        bi = mu_min_norm(3*i-2:3*i);
        bj = mu_min_norm(3*j-2:3*j);
        Pbari = Pbar(3*i-2:3*i,:);
        Pbarj = Pbar(3*j-2:3*j,:);
        li = cable_length(i);
        lj = cable_length(j);
        rhoi = rho_mat(:,i);
        rhoj = rho_mat(:,j);
        
        unit_null_mu_i = (bi + Pbari*x)/norm(bi + Pbari*x);
        unit_null_mu_j = (bj + Pbarj*x)/norm(bj + Pbarj*x);
        
        norm_of_dist_between_robots = norm(li*unit_null_mu_i - lj*unit_null_mu_j + rhoi - rhoj);
        y(constraint_idx) = 2*robot_radius - norm_of_dist_between_robots;
    
    end
end

% y = zeros(1,jj);
% for i = 1:jj
%     y(i) = 1/2*x'*H{i}*x + k{i}'*x + d{i};
% end

yeq = [];

if nargout > 2
    grady = zeros(length(x),length(y));
    grad_idx = 0;
    
    for i = 1:num_of_robots
        for j = i+1:num_of_robots
            
        end
    end
    
end

gradyeq = [];