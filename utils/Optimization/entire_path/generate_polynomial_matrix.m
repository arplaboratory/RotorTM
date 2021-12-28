function matrix = generate_polynomial_matrix(max_exponent,max_diff,var)


matrix = [];

for i = 0:max_diff
    polynomial = [];
    for j = 0:max_exponent
        
        exponent = j - i;
        if exponent >= 0
            term = var^exponent;
        else
            term = 0;
        end
        polynomial = [polynomial term];
        
    end
    matrix = [matrix;polynomial];
end

end