#! /usr/bin/env python

import numpy as np
from utils.Optimization.entire_path.generate_poly import generate_poly
import scipy

# test passed

def generate_constaint(path, traj_constant):
    max_exponent = traj_constant.max_exponent
    max_diff = traj_constant.max_diff
    traj_num = path.shape[0]-1
    max_vel = traj_constant.max_vel
    max_acc = traj_constant.max_acc
    num_coeff = traj_constant.num_coeff
    dim = traj_constant.dim

    vel_limit = np.array([[max_vel, max_vel, max_vel]])
    accel_limit = np.array([[max_acc, max_acc, max_acc]])

    # these init may cause error
    A = np.zeros((traj_num*dim*num_coeff,traj_num*dim*num_coeff),dtype=float)
    Aeq = np.array([[]])
    Aeq_pos = np.zeros((traj_num*dim*2, traj_num*dim*num_coeff), dtype=float)
    Aeq_vel = np.zeros(((traj_num+1)*dim, traj_num*dim*num_coeff), dtype=float)
    Aeq_acc = np.zeros(((traj_num+1)*dim, traj_num*dim*num_coeff), dtype=float)
    b = np.zeros((traj_num*dim*num_coeff,1),dtype=float)
    beq = np.array([[]])
    beq_pos = np.zeros((traj_num*dim*2, 1), dtype=float)
    beq_vel = np.zeros(((traj_num+1)*dim, 1), dtype=float)
    beq_acc = np.zeros(((traj_num+1)*dim, 1), dtype=float)

# vel and acc constraints on the waypoints in the path

    for j in range(1, traj_num+1):
        
        pos_constraint_start = path[j-1:j,0:3]
        pos_constraint_end   = path[j:j+1,0:3]
        
        t_start = path[j-1,3]
        t_end   = path[j  ,3]
        
        # generate polynomial
        poly_coeff_start = generate_poly(max_exponent,max_diff,t_start)
        poly_coeff_end = generate_poly(max_exponent,max_diff,t_end)
        
        # pos constraints on jth trajectory(between the jth waypoint and j+1th waypoint) in the path
        Aeq_pos_j = np.append(scipy.linalg.block_diag(poly_coeff_start[0,:], poly_coeff_start[0,:], poly_coeff_start[0,:]), scipy.linalg.block_diag(poly_coeff_end[0,:], poly_coeff_end[0,:], poly_coeff_end[0,:]), axis=0)
        beq_pos_j = np.append(np.transpose(pos_constraint_start), np.transpose(pos_constraint_end), axis=0)
        Aeq_pos[((j-1)*dim*2):(j*dim*2),((j-1)*dim*num_coeff):(j*dim*num_coeff)] = Aeq_pos_j
        beq_pos[(j-1)*dim*2:j*dim*2,0:1] = beq_pos_j
        
        # vel polynomial terms on the jth waypoint in the path, constructed in
        # the matrix Aeq
        Aeq_vel_j_start = scipy.linalg.block_diag(poly_coeff_start[1,:],poly_coeff_start[1,:],poly_coeff_start[1,:])
        
        # acc polynomial terms on the jth waypoint in the path, constructed in
        # the matrix Aeq
        Aeq_acc_j_start = scipy.linalg.block_diag(poly_coeff_start[2,:],poly_coeff_start[2,:],poly_coeff_start[2,:])
        
        if j == 1:
            
            #vel and acc constraints on the 1st waypoint in the path
            beq_vel_j_start = np.zeros((dim,1), dtype=float)
            beq_acc_j_start = np.zeros((dim,1), dtype=float)
            
            Aeq_vel[(j-1)*dim:j*dim,(j-1)*dim*num_coeff:j*dim*num_coeff] = Aeq_vel_j_start
            Aeq_acc[(j-1)*dim:j*dim,(j-1)*dim*num_coeff:j*dim*num_coeff] = Aeq_acc_j_start
            beq_vel[(j-1)*dim:j*dim,0:1] = beq_vel_j_start
            beq_acc[(j-1)*dim:j*dim,0:1] = beq_acc_j_start
            
        else:
            
            #vel and acc constraints on the jth waypoint
            #for continuity of the j-1th trajectory and the jth trajectory
            beq_vel_j_start = np.zeros((dim,1), dtype=float)
            beq_acc_j_start = np.zeros((dim,1), dtype=float)
            
            Aeq_vel[(j-1)*dim:j*dim,(j-2)*dim*num_coeff:j*dim*num_coeff] = np.append(Aeq_vel_j_start, -Aeq_vel_j_start, axis = 1)
            Aeq_acc[(j-1)*dim:j*dim,(j-2)*dim*num_coeff:j*dim*num_coeff] = np.append(Aeq_acc_j_start, -Aeq_acc_j_start, axis = 1)
            beq_vel[(j-1)*dim:j*dim,0:1] = beq_vel_j_start
            beq_acc[(j-1)*dim:j*dim,0:1] = beq_acc_j_start
            
        #vel and acc constraints on the end waypoint
        #for stopping at the goal position   
        if j == traj_num:
            
            Aeq_vel_j_end = scipy.linalg.block_diag(poly_coeff_end[1,:],poly_coeff_end[1,:],poly_coeff_end[1,:])
            Aeq_acc_j_end = scipy.linalg.block_diag(poly_coeff_end[2,:],poly_coeff_end[2,:],poly_coeff_end[2,:])
            beq_vel_j_end = np.zeros((dim,1), dtype=float)
            beq_acc_j_end = np.zeros((dim,1), dtype=float)
            
            Aeq_vel[j*dim:(j+1)*dim,(j-1)*dim*num_coeff:j*dim*num_coeff] = Aeq_vel_j_end
            Aeq_acc[j*dim:(j+1)*dim,(j-1)*dim*num_coeff:j*dim*num_coeff] = Aeq_acc_j_end
            beq_vel[j*dim:(j+1)*dim,0:1] = beq_vel_j_end
            beq_acc[j*dim:(j+1)*dim,0:1] = beq_acc_j_end

    Aeq = np.vstack((Aeq_pos, Aeq_vel, Aeq_acc))
    beq = np.vstack((beq_pos, beq_vel, beq_acc))

    return A, b, Aeq, beq

