import numpy as np
    
def rigid_links_cooperative_payload_controller(ql = None,params = None): 
    # CONTROLLER quadrotor controller
# The current states are:
# qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
# The desired states are:
# qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
# Using these current and desired states, you have to compute the desired controls
    
    # =================== Your code goes here ===================
    
    
    if (not params.sim_start ):
        icnt = 0
    
    icnt = icnt + 1
    ## Parameter Initialization
    quat_des = ql.quat_des
    yaw_des = 0
    omega_des = ql.omega_des
    g = params.grav
    m = params.struct_mass
    e3 = np.array([[0],[0],[1]])
    Rot = ql.rot
    omega = ql.omega
    ## Position control
#jerk_des = ql.jerk_des;
#Position error
    ep = ql.pos_des - ql.pos
    #Velocity error
    ed = ql.vel_des - ql.vel
    # Desired acceleration This equation drives the errors of trajectory to zero.
    acceleration_des = ql.acc_des + params.Kp * ep + params.Kd * ed
    # Net force F=kx*ex kv*ex_dot + mge3 +mxdes_ddot
    Force = m * g * e3 + m * acceleration_des
    tau = np.transpose(Force) * Rot * e3
    Rot_des = np.zeros((3,3))
    Z_body_in_world = Force / norm(Force)
    Rot_des[:,3] = Z_body_in_world
    X_unit = np.array([[np.cos(yaw_des)],[np.sin(yaw_des)],[0]])
    Y_body_in_world = cross(Z_body_in_world,X_unit)
    Y_body_in_world = Y_body_in_world / norm(Y_body_in_world)
    Rot_des[:,2] = Y_body_in_world
    X_body_in_world = cross(Y_body_in_world,Z_body_in_world)
    Rot_des[:,1] = X_body_in_world
    ## Attitude Control
    
    # Errors of anlges and angular velocities
    e_Rot = np.transpose(Rot_des) * Rot - np.transpose(Rot) * Rot_des
    e_angle = vee(e_Rot) / 2
    e_omega = omega - np.transpose(Rot) * Rot_des * np.transpose(omega_des)
    # Net moment
# Missing the angular acceleration term but in general it is neglectable.
    M = - params.Kpe * e_angle - params.Kde * e_omega + cross(omega,params.struct_I * omega)
    ## Quadrotor Thrust and Moment Distribution
    u = params.thrust_moment_distribution_mat * np.array([[tau],[M]])
    return u
    
    return u