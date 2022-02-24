#! /usr/bin/env python

import numpy as np
# test passed
def generate_path_with_time(path=None, traj_constant=None, T_seg=None, t_start=None):
    if (np.any(path!= None) ) and (np.any(traj_constant != None)) and (np.any(T_seg != None)) and (np.any(t_start != None)):
        time = t_start
        time_list = np.array([[time]])
        traj_num = T_seg.shape[0] # may be shape[1]

        for j in range(1, traj_num+1):
            time = time + T_seg[j-1]
            time_array = np.array([[time]])
            time_list = np.append(time_list, time_array, axis=0)        
            
        path_with_time = np.append(path, time_list, axis=1)
        timelist = path_with_time[:, 3]

    elif (np.any(path!= None) ) and (np.any(traj_constant != None)) and (np.any(T_seg != None)):
        time = 0.0
        time_list = np.array([[time]])

        for j in range(1, traj_constant.total_traj_num+1):
            time = time + T_seg[j-1]
            time_array = np.array([[time]])
            time_list = np.append(time_list, time_array, axis=0)

        path_with_time = np.append(path, time_list, axis=1)
        timelist = path_with_time[:, 3:]

    return path_with_time,timelist