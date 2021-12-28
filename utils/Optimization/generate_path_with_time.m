function [path_with_time timelist] = generate_path_with_time(path,traj_constant,T_seg,t_start)

if nargin == 4
    
    time = t_start;
    time_list = [time];
    traj_num = length(T_seg);
    
    for j = 1:traj_num
        time = time+T_seg(j);
        time_list = [time_list;time];
    end
    
    path_with_time = [path time_list];
    timelist = path_with_time(:,4);
    
elseif nargin == 3
    
    time = 0.0;
    time_list = [time];
    %traj_num = length(T_seg);
    
    for j = 1:traj_constant.total_traj_num
        time = time+T_seg(j);
        time_list = [time_list;time];
    end
    
    path_with_time = [path time_list];
    timelist = path_with_time(:,4);
    
else
    error('The number of input arguments are not correct');
end



end