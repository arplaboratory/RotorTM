import numpy as np
    
def cooperative_attitude_controller(qd = None,qn = None,params = None): 
    # CONTROLLER quadrotor controller
# The current states are:
# qd.pos, qd.vel, qd.euler = [roll;pitch;yaw], qd.omega
# The desired states are:
# qd.pos_des, qd.vel_des, qd.acc_des, qd.yaw_des, qd.yawdot_des
# Using these current and desired states, you have to compute the desired controls
# INPUTS:
# qd     - 1 x 1, Quads' states
# qn     - Quad's id
# params - struct, output from crazyflie() and whatever parameters you want to pass in
    
    # OUTPUTS:
# F      - 1 x 1, thrust output from controller (only used in simulation)
# M      - 3 x 1, moments output from controller (only used in simulation)
# =================== Your code goes here ===================
    
    
    
    if len(gd)==0:
        gd = np.zeros((0,3))
        icnt = 0
    
    icnt = icnt + 1
    ## Parameter Initialization
    m = params.mass
    l = params.l
    e3 = np.array([[0],[0],[1]])
    ## State Feedback
    xi_ = qd[qn].xi
    xidot_ = qd[qn].xidot
    rot = qd[qn].rot
    ## Cable Direction Tracking Control
    mu_des_ = qd[qn].mu_des
    xi_des_ = - mu_des_ / norm(mu_des_)
    xi_des_dot_ = np.array([[0],[0],[0]])
    
    w_des_ = cross(xi_des_,xi_des_dot_)
    w_ = cross(xi_,xidot_)
    mu_ = qd[qn].xixiT * mu_des_
    e_xi = cross(xi_des_,xi_)
    e_w = w_ + cross(xi_,cross(xi_,w_des_))
    u_parallel = mu_ + m * l * norm(w_) ** 2 * xi_ + m * qd[qn].xixiT * qd[qn].attach_accel
    u_perpendicular = - m * l * cross(xi_,params.Kxi * e_xi + params.Kw * e_w + (np.transpose(xi_) * w_des_) * xidot_) - m * cross(xi_,cross(xi_,qd[qn].attach_accel))
    Force = u_parallel + u_perpendicular
    F = np.transpose(Force) * rot * e3
    ## Desired Attitude and Angular Velocity
    yaw_des = qd[qn].yaw_des
    yawdot_des = qd[qn].yawdot_des
    Rot_des = np.zeros((3,3))
    Z_body_in_world = Force / norm(Force)
    Rot_des[:,3] = Z_body_in_world
    X_unit = np.array([[np.cos(yaw_des)],[np.sin(yaw_des)],[0]])
    Y_body_in_world = cross(Z_body_in_world,X_unit)
    Y_body_in_world = Y_body_in_world / norm(Y_body_in_world)
    Rot_des[:,2] = Y_body_in_world
    X_body_in_world = cross(Y_body_in_world,Z_body_in_world)
    Rot_des[:,1] = X_body_in_world
    #p_des = -(m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*Y_body_in_world;
#q_des = (m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*X_body_in_world;
    p_des = 0.0
    q_des = 0.0
    r_des = yawdot_des * Z_body_in_world(3)
    qd[qn].rot_des = Rot_des
    qd[qn].omega_des = np.transpose(np.array([p_des,q_des,r_des]))
    ## Quadrotor Attitude Control
    M = quadrotor_attitude_controller(qd[qn],params)
    # =================== Your code ends here ===================
    
    #Output trpy and drpy as in hardware
    trpy = np.array([0,0,0,0])
    drpy = np.array([0,0,0,0])
    return F,M,trpy,drpy
    
    return F,M,trpy,drpy