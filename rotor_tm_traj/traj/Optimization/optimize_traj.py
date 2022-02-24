#! /usr/bin/env python

import numpy as np
from Optimization.generate_path_with_time import generate_path_with_time
from Optimization.entire_path.generate_H import generate_H
from Optimization.generate_constraint import generate_constaint
from Optimization.generate_corridor_constraint import generate_corridor_constraint
from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp

# quadprog still not working
# needs rest is working fine
# need to find suitable python quadprog solver

def optimize_traj(finalpath, traj_constant, T_seg_all, cor_constraint):
    total_traj_num = traj_constant.total_traj_num
    traj_num = traj_constant.traj_num
    pt_num = traj_constant.pt_num
    dim = traj_constant.dim
    num_coeff = traj_constant.num_coeff
    coeff = np.array([[]])
    timelist = np.array([[]])
    path_with_time_all, timelist_all = generate_path_with_time(finalpath, traj_constant, T_seg_all)


    for i in range(1, pt_num-1, total_traj_num+2): 
        if (i+pt_num-1) >= total_traj_num+1:
            path = finalpath[i-1:total_traj_num+1, :]
            T_seg = T_seg_all[i-1:total_traj_num]
        else:
            path = finalpath[i-1:i+traj_num, :]
            T_seg = T_seg_all[i-1:i+traj_num-1]
    
        if (i == total_traj_num+1):
            coefficient = coeff.reshape((num_coeff, total_traj_num*dim))
            return coefficient, timelist_all

        path_with_time, timelist_i = generate_path_with_time(path, traj_constant, T_seg, 0)

        # Generate Quadratic Weight matrix
        H_result = generate_H(traj_constant, timelist_i)

        # Generate constraint for conitinuity and max and min limit 
        A, b, Aeq, beq = generate_constaint(path_with_time, traj_constant)

        # Generate corridor constraint
        A_corr, b_corr = generate_corridor_constraint(cor_constraint, path_with_time, traj_constant)
        A = np.append(A, A_corr, axis=0)
        b = np.append(b, b_corr, axis=0)

        # Solve the quadrotor programming
        qp_sol = qp(matrix(H_result), matrix(np.zeros(H_result.shape[0])), matrix(A), matrix(b), matrix(Aeq), matrix(beq))

        # Append the coefficients
        coeff_curr = np.array(qp_sol['x'])
        coeff = np.append(coeff, coeff_curr)

    coefficient = coeff.reshape((total_traj_num*dim, num_coeff)).T
    return coefficient, timelist_all
        