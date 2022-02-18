function mat = vec2asym(vec)
% function mat = vec2asym(vec)
%   The vee function maps a 3-d vector to 
%   a 3x3 skew-symmetric matrix

[row,col] = size(vec); 

if(((row==3)&&(col==1)) || ((row==1)&&(col==3)))
    mat = [0,-vec(3),vec(2);
           vec(3),0,-vec(1);
           -vec(2),vec(1),0];
else
    error('The input for this function should be 3x1 or 1x3 vector');
end

end