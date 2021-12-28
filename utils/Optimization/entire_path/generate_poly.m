function f = generate_poly(max_exponent,max_diff,symbol)

for k = 0:max_diff
    for i = 0:max_exponent
        if (i - k) >= 0
            f(k+1,i+1)  = factorial(i)*symbol^(i-k)/factorial(i-k);
            %f{i+1} = int(f_i,symbol,lower_limit,upper_limit);
        else 
            f(k+1,i+1) = 0;
        end
    end
end

end