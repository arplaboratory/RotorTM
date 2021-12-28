function [T_seg] = optimal_time_seg(finalpath,traj_constant,T_seg_c,cor_constraint,options)

h = 1e-01;
gamma = 0.1;
eps = 1e-06;
time_optimization = false;
fprintf('The original function value is %d.\n',compute_func(finalpath,traj_constant,T_seg_c,cor_constraint,options));

while ~time_optimization
    T_seg_temp = [];
    f_val_temp = [];
    
    %     g = zeros(1,traj_constant.traj_num);
    
    for i = 1:traj_constant.traj_num
        %     [path_with_time, timelist] = generate_path_with_time(finalpath,traj_constant,T_seg);
        %     H = generate_H(traj_constant,timelist);
        %
        %     %generate constraints
        %     [A,b,Aeq,beq] = generate_constraint(path_with_time,traj_constant);
        %     [A_corr,b_corr] = generate_corridor_constraint(cor_constraint,path_with_time,traj_constant);
        %     A = [A;A_corr];
        %     b = [b;b_corr];
        %
        %     [coeff, fval, exitflag] = quadprog(H,[],A,b,Aeq,beq,[],[],[],options);
        gi = ones(1,traj_constant.traj_num)/(1-traj_constant.traj_num);
        gi(i) = 1;
        %         g = g + gi;
        fval_c = compute_func(finalpath,traj_constant,T_seg_c,cor_constraint,options);
        fval_nc = compute_func(finalpath,traj_constant,T_seg_c+h*gi,cor_constraint,options);
        gradient = (fval_nc - fval_c) / (h*norm(gi));
        %disp('Computed the gradient ')
        if gradient < 0
            d = 0.1;
            T_seg_n      =   T_seg_c+d*gi;
            cgradient  =   d*gradient;
            fval_n     =   compute_func(finalpath,traj_constant,T_seg_n,cor_constraint,options);
            fcall   =   1 ;
            while fval_n > fval_c+cgradient
                
                d       =  gamma*d;
                cgradient  =  gamma*cgradient;
                T_seg_n      =   T_seg_c+d*gi;
                fval_n      =  compute_func(finalpath,traj_constant,T_seg_n,cor_constraint,options);
                fcall   =  fcall+1;
                
                %Check if the step to xn is too small.
                if norm(d) <= eps
                    %fprintf('linesearch step too small when i = %d\n', i)
                    if fval_n >= fval_c
                        T_seg_n   =   T_seg_c;
                    end
                    break
                end
            end
            T_seg_temp = [T_seg_temp;T_seg_n];
            f_val_temp = [f_val_temp;fval_n];
        else
            fprintf('gradient is larger than zero when i = %d\n', i)
        end
        
    end
    
    if ~isempty(T_seg_temp)
        [M,I] = min(f_val_temp);
        T_seg_c = T_seg_temp(I,:);
        fprintf('The new value of the function is %d.\n',compute_func(finalpath,traj_constant,T_seg_c,cor_constraint,options));
    else
        time_optimization = true;
    end
    
end

T_seg = T_seg_c;

end