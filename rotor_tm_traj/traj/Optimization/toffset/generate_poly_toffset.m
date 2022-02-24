function A  = generate_poly_toffset(max_exponent,max_diff,t,t_start)

A = [];

for k = 0:max_diff
    for i = 0:max_exponent
        if (i - k) >= 0
            A(k+1,i+1)  = factorial(i)*(t-t_start)^(i-k)/factorial(i-k);
            %f{i+1} = int(f_i,symbol,lower_limit,upper_limit);
        else
            A(k+1,i+1) = 0;
        end
    end
end


end