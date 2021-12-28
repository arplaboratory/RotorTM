function poly_coeff = generate_poly_coeff(traj_constant)

max_exponent = traj_constant.max_exponent;
max_diff = traj_constant.max_diff;

for k = 0:max_diff
    for i = 0:max_exponent
        if (i - k) >= 0
            poly_coeff(k+1,i+1)  = factorial(i)/factorial(i-k);
            %f{i+1} = int(f_i,symbol,lower_limit,upper_limit);
        else 
            poly_coeff(k+1,i+1) = 0;
        end
    end
end

end