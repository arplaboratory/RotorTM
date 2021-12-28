function [coefficient,timelist_all] = optimize_traj(finalpath,traj_constant,T_seg_all,cor_constraint,options)
%% Initialization
total_traj_num = traj_constant.total_traj_num;
traj_num = traj_constant.traj_num;
pt_num = traj_constant.pt_num;
dim = traj_constant.dim;
num_coeff = traj_constant.num_coeff;
coeff = [];
timelist = [];
[path_with_time_all, timelist_all] = generate_path_with_time(finalpath,traj_constant,T_seg_all);

%% 

%options = optimoptions('quadprog','Display','iter','MaxIter',200,'TolFun',1e-6,'TolX',1e-6,'TolCon',1e-6);
for i = 1:pt_num-1:total_traj_num+1
    
    %fprintf("The index is %d\n", i);
    if i+pt_num-1 >= total_traj_num+1
        path = finalpath(i:total_traj_num+1,:);
        T_seg = T_seg_all(i:total_traj_num);
    else
        path = finalpath(i:i+traj_num,:);
        T_seg = T_seg_all(i:i+traj_num-1);
    end
    
    if i == total_traj_num+1
        break;
    end
    
    [path_with_time, timelist_i] = generate_path_with_time(path,traj_constant,T_seg,0);
    H = generate_H(traj_constant,timelist_i);
    %H = generate_H_with_toffset(traj_constant,timelist);
    
    %generate constraints
    [A,b,Aeq,beq] = generate_constraint(path_with_time,traj_constant);
    [A_corr,b_corr] = generate_corridor_constraint(cor_constraint,path_with_time,traj_constant);
    A = [A;A_corr];
    b = [b;b_corr];
    
    [coeff_curr, fval, exitflag] = quadprog(H,[],A,b,Aeq,beq,[],[],[],options);
    
    coeff = [coeff;coeff_curr];
    %timelist = [timelist;timelist_i];
    
end

coefficient = reshape(coeff,[num_coeff,total_traj_num*dim]);

end
