function time_segment = allocate_time(path,max_vel,max_acc)

[pathlength ~] = size(path);

distance = path(2:pathlength,:) - path(1:pathlength-1,:);
distance = vecnorm(distance',2,1);

time_segment = 3.0*(distance*2/max_vel) .* (1 + exp(-2*distance/max_vel)* 6.5 * max_vel/max_acc);

end