#! /usr/bin/env python

import math as m
import numpy as np

# test passed

def generate_poly_coeff(traj_constant):
 
    max_exponent = traj_constant.max_exponent
    max_diff = traj_constant.max_diff
    poly_coeff = np.zeros((max_diff+1, max_exponent+1), dtype=float)

    for k in range(max_diff+1):
        for i in range(max_exponent+1):
            if (i - k) >= 0:
                poly_coeff[k,i]  = m.factorial(i)/m.factorial(i-k)
                # f{i+1} = int(f_i,symbol,lower_limit,upper_limit)
            else:
                poly_coeff[k,i] = 0

    return poly_coeff