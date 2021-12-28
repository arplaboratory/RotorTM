%function H = generate_H(max_exponent,max_diff,timelist)
function H = generate_H(traj_constant,timelist)

max_exponent = traj_constant.max_exponent;
max_diff = traj_constant.max_diff;
trajs_num = length(timelist)-1;
H = [];

for time_index = 1:trajs_num
    
    lower_limit = timelist(time_index);
    upper_limit = timelist(time_index+1);

    for i = 0:max_exponent
        for j = 0:max_exponent
            if (i - max_diff) >= 0 && (j - max_diff) >= 0
               const1 = i + j - 2*max_diff + 1;
               H_seg(i+1,j+1) = factorial(i)*factorial(j)*(upper_limit^const1-lower_limit^const1)/(factorial(i - max_diff)*factorial(j - max_diff)*const1);
            end
        end
    end
   
    Hi = blkdiag(H_seg,H_seg,H_seg);
    H = blkdiag(H,Hi);
    
end

end