from unittest import result
import numpy as np
from Optimization.generate_2pt_corridor_constraint import generate_2pt_corridor_constraint
# tested


def generate_corridor_constraint(cor_constraint, path_with_time, traj_constant):
    traj_num = path_with_time.shape[0] - 1

    A = np.empty((traj_num * traj_constant.nc * traj_constant.dim * 2 + 1, traj_num * traj_constant.dim * traj_constant.num_coeff))
    b = np.empty((traj_num * traj_constant.nc * traj_constant.dim * 2 + 1, 1))

    if cor_constraint:
        for i in range(1, traj_num + 1):
            path = path_with_time[i - 1:i + 1, :]
            A_corr, b_corr = generate_2pt_corridor_constraint(path, traj_constant)

            row_lower_ind = (i - 1) * traj_constant.nc * traj_constant.dim * 2 + 1
            row_upper_ind = i * traj_constant.nc * traj_constant.dim * 2
            col_lower_ind = (i - 1) * traj_constant.dim * traj_constant.num_coeff + 1
            col_upper_ind = i * traj_constant.dim * traj_constant.num_coeff

            A[row_lower_ind - 1:row_upper_ind, col_lower_ind - 1:col_upper_ind] = A_corr
            b[row_lower_ind - 1:row_upper_ind, 0:1] = b_corr

    else:
        A = np.zeros((0, traj_num * traj_constant.dim * traj_constant.num_coeff), dtype=float)

        b = np.zeros((0, 1), dtype=float)
    return A, b
