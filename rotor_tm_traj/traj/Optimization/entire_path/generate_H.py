#! /usr/bin/env python

from matplotlib.pyplot import sci
import numpy as np
import math as m
from scipy import linalg
# test pass
def generate_H (traj_constant, timelist):
    max_exponent = traj_constant.max_exponent
    max_diff = traj_constant.max_diff
    trajs_num = timelist.shape[0]-1
    H = np.zeros((0,0))

    H_seg = np.zeros((max_exponent+1, max_exponent+1), dtype=float)
    for time_index in range(1, trajs_num+1):
        lower_limit = timelist[time_index-1]
        upper_limit = timelist[time_index]
    
        for i in range(max_exponent+1):
            for j in range(max_exponent+1):
                if (i - max_diff) >= 0 and (j - max_diff) >= 0:
                    const1 = i + j - 2*max_diff + 1
                    H_seg[i, j] = m.factorial(i)*m.factorial(j)*(upper_limit**const1-lower_limit**const1)/(m.factorial(i-max_diff)*m.factorial(j-max_diff)*const1)

        Hi = linalg.block_diag(H_seg, H_seg, H_seg)
        H = linalg.block_diag(H, Hi)
    return H
