function [A,b] = generate_corridor_constraint(cor_constraint,path_with_time,traj_constant)

traj_num = size(path_with_time,1)-1;

if (cor_constraint)
    
    for i = 1:traj_num
        
        path = path_with_time(i:i+1,:);
        %if  i == 2 || i ==1
        [A_corr,b_corr] = generate_2pt_corridor_constraint(path,traj_constant);
        %row_lower_ind = (i-1)*traj_constant.nc*traj_constant.num_coeff+1;
        %row_upper_ind = i*traj_constant.nc*traj_constant.num_coeff;
        row_lower_ind = (i-1)*traj_constant.nc*traj_constant.dim*2+1;
        row_upper_ind = i*traj_constant.nc*traj_constant.dim*2;
        col_lower_ind = (i-1)*traj_constant.dim*traj_constant.num_coeff+1;
        col_upper_ind = i*traj_constant.dim*traj_constant.num_coeff;
        A(row_lower_ind:row_upper_ind,col_lower_ind:col_upper_ind) = A_corr;
        b(row_lower_ind:row_upper_ind,1) = b_corr;
        %end
        
    end
    
else
    
    A = [];
    b = [];
    
end



end