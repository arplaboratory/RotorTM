import numpy as np
    
def cooperative_payload_geometric_controller(ql = None,qd = None,pl_params = None,qd_params = None): 
    # CONTROLLER quadrotor controller
# The current states are:
# qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
# The desired states are:
# qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
# Using these current and desired states, you have to compute the desired controls
    
    # =================== Your code goes here ===================
    
    
    
    
    if (not pl_params.sim_start ):
        coeff0 = pl_params.coeff0
        icnt = 0
    
    icnt = icnt + 1
    ## Parameter Initialization
    quat_des = ql.quat_des
    omega_des = ql.omega_des
    g = pl_params.grav
    m = pl_params.mass
    nquad = pl_params.nquad
    e3 = np.array([[0],[0],[1]])
    Rot = ql.rot
    omega_asym = vec2asym(ql.omega)
    Rot_des = np.transpose(QuatToRot(quat_des))
    ## Position control
#Position error
    ep = ql.pos_des - ql.pos
    #Velocity error
    ed = ql.vel_des - ql.vel
    # Desired acceleration This equation drives the errors of trajectory to zero.
    acceleration_des = ql.acc_des + pl_params.Kp * ep + pl_params.Kd * ed
    # Net force F=kx*ex kv*ex_dot + mge3 +mxdes_ddot
    F = m * g * e3 + m * acceleration_des
    ## Attitude Control
    
    # Errors of anlges and angular velocities
    e_Rot = np.transpose(Rot_des) * Rot - np.transpose(Rot) * Rot_des
    e_angle = vee(e_Rot) / 2
    e_omega = ql.omega - np.transpose(Rot) * Rot_des * np.transpose(omega_des)
    # Net moment
# Missing the angular acceleration term but in general it is neglectable.
    M = - pl_params.Kpe * e_angle - pl_params.Kde * e_omega
    ## Cable tension distribution
    diag_rot = []
    for i in np.arange(1,nquad+1).reshape(-1):
        diag_rot = blkdiag(diag_rot,Rot)
    
    mu = diag_rot * pl_params.pseudo_inv_P * np.array([[np.transpose(Rot) * F],[M]])
    for i in np.arange(1,nquad+1).reshape(-1):
        mu[3 * i] = np.amax(0,mu(3 * i))
    
    att_acc_c = acceleration_des + g * e3 + Rot * omega_asym * omega_asym * pl_params.rho_vec_list
    ## Cable optimal distribution
# P_bar_world = diag_rot * params.P_bar;
# rho_vec_world = Rot * params.rho_vec_list;
# [mu, nullspace_coeff] = optimal_cable_distribution(P_bar_world, rho_vec_world, mu, params, coeff0);
# coeff0 = nullspace_coeff;
    
    ## Quadrotor Attitude Controller
    for qn in np.arange(1,nquad+1).reshape(-1):
        qd[qn].yaw_des = 0
        qd[qn].yawdot_des = 0
        qd[qn].mu_des = mu(np.arange(3 * qn - 2,3 * qn+1))
        qd[qn].attach_accel = att_acc_c(:,qn)
        F_qn,M_qn,trpy,drpy = cooperative_attitude_controller(qd,qn,qd_params)
        qd_F[qn] = F_qn
        qd_M[qn] = M_qn
    
    return qd_F,qd_M
    
    return qd_F,qd_M