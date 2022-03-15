# Add additional initialization if you want to.
# You can use this space to pre-compute the trajectory to avoid
# repeatedly computing the same trajectory in every call of the
# "trajectory_generator" function

# Generate trajectory

import numpy as np
print('Generating Trajectory ...')
#ttotal = 10;

cor_constraint = True
#plot_path(map, path{1});

# trajectory_generator([], [], map, path);
# basicdata  = map.basicdata;
# [rowbasicdata ~] = size(basicdata);`
# if rowbasicdata >= 2
#     block = basicdata(2:rowbasicdata,:);
# else
#     block = [];
# end
#finalpath = simplify_path(path{1},block,map);
#finalpath = simplify_path_with_ellipsoid_cor(path{1},ellipsoid_corridor)

#min_snap_traj_generator([], [], map, path, cor_constraint);
#simplify_path_with_ellipsoid_cor(path{1},ellipsoid_corridor)

## Initialize circle trajectory
payload_start = np.array([np.array([0.0,0.0,0.0,1.0,0.0,0.0,0.0])])
payload_stop = np.array([np.array([1.0,1.0,1.0,1.0,0.0,0.0,0.0])])
radius = 1.0
period = 10
duration = 10
__,payload_stop = circle([],[],payload_start[0](np.arange(1,3+1)),radius,period,duration)
#jump([], [], payload_start{1}(1:3),payload_stop{1}(1:3))
## Initialize jump trajectory
# payload_start = {[0.0 0.0 0.0 1.0 0.0 0.0 0.0]};
# payload_stop = {[0.0 0.0 1.0 1.0 0.0 0.0 0.0]};
# jump([], [],payload_start{1},payload_stop{1});