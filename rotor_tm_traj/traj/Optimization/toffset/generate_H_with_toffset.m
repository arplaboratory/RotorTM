function H = generate_H_with_toffset(max_exponent,max_diff,timelist)
%%
%The output 
%H - H = diag(H1, H2, ... , H_trajs_num)
%    where Hk is a matrix with the size of (max_diff+1)x(max_diff+1) 
%    The index k corresponds to the kth trajectory in the planned path,
%    which is from t_k to t_{k+1}
%    Each element in the matrix Hk is Hijk
%    Hijk = i!*j!*(t_{k+1} - t_k)^(i+j-2*max_diff+1)/(i-max_diff)!*(j-max_diff)!*(i+j-2m+1)
%    Also Hijkx = Hijky = Hijkz = Hijk
%%
trajs_num = length(timelist)-1;
H = zeros((max_exponent+1)*trajs_num);

for time_index = 1:trajs_num
    
    lower_limit = timelist(time_index);
    upper_limit = timelist(time_index+1);
    delta_time = upper_limit - lower_limit;

    for i = 0:max_exponent
        for j = 0:max_exponent
            if (i - max_diff) >= 0 && (j - max_diff) >= 0
               const1 = i + j - 2*max_diff + 1;
               %H_seg(i+1,j+1) = int(f{i+1,time_index} * f{j+1,time_index},symbol,lower_limit,upper_limit);
               H_seg(i+1,j+1) = factorial(i)*factorial(j)*delta_time^const1/(factorial(i - max_diff)*factorial(j - max_diff)*const1);
            end
        end
    end
    
    lower_index = (max_exponent+1)*time_index - max_exponent;
    upper_index = (max_exponent+1)*time_index;
    H(lower_index:upper_index,lower_index:upper_index) = H_seg;
    
end

end