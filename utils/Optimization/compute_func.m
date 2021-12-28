function [fval, coeff,timelist] = compute_func(finalpath,traj_constant,T_seg,cor_constraint,options)


    [path_with_time, timelist] = generate_path_with_time(finalpath,traj_constant,T_seg);
    H = generate_H(traj_constant,timelist);
    
    %generate constraints
    [A,b,Aeq,beq] = generate_constraint(path_with_time,traj_constant);
    [A_corr,b_corr] = generate_corridor_constraint(cor_constraint,path_with_time,traj_constant);
    A = [A;A_corr];
    b = [b;b_corr];
    
    [coeff, fval, exitflag] = quadprog(H,[],A,b,Aeq,beq,[],[],[],options);


end
