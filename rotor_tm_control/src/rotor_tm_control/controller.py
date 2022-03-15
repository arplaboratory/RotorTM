#!/usr/bin/python3
import numpy as np
from rotor_tm_utils.vec2asym import vec2asym
from rotor_tm_utils.QuatToRot import QuatToRot
import scipy.linalg as LA
from rotor_tm_utils.vee import vee
from rotor_tm_utils.RPYtoRot_ZXY import RPYtoRot_ZXY
from rotor_tm_utils import utilslib 
from scipy.spatial.transform import Rotation as tranrot


class controller:
    def __init__(self):
        self.gd = np.zeros((0,0), dtype=float)
        self.icnt = None
        
        # for hover_controller
        self.last_t = None

    # test passed
    def cooperative_attitude_controller(self, qd, qn, params):
        if self.gd.size == 0:
            self.gd = np.zeros((0,3), dtype= float)
            self.icnt = 0
        
        # Parameter Initialization
        m = params.mass
        l = params.l
        e3 = np.array([[0],[0],[1]])

        # State Feedback
        xi = qd[qn]["xi"]
        xidot = qd[qn]["xidot"]
        rot = qd[qn]["rot"]

        # Cable Direction Tracking Control
        mu_des = qd[qn]["mu_des"]
        xi_des = np.divide(-mu_des, np.linalg.norm(mu_des))
        xi_des_dot = np.array([[0.0],[0.0],[0.0]])
        w_des = np.cross(xi_des, xi_des_dot, axisa=0, axisb=0).T
        
        w = np.cross(xi, xidot, axisa=0, axisb=0).T
        mu = np.matmul(qd[qn]["xixiT"], mu_des)
        e_xi = np.cross(xi_des, xi, axisa=0, axisb=0).T
        e_w = w + np.cross(xi, np.cross(xi, w_des, axisa=0, axisb=0).T, axisa=0, axisb=0).T

        u_parallel = mu + m*l*np.linalg.norm(w)**2*xi + np.matmul(m*qd[qn]["xixiT"], qd[qn]["attach_accel"])
        u_perpendicular = -m*l*np.cross(xi, params.Kxi @ e_xi + params.Kw @ e_w + (xi.T @ w_des) * xi_des_dot, axisa=0, axisb=0).T - m*np.cross(xi, np.cross(xi, qd[qn]["attach_accel"], axisa=0, axisb=0).T, axisa=0, axisb=0).T
        Force = u_parallel + u_perpendicular
        #F = np.dot(Force,np.matmul(rot,e3))
        F = Force.T @ np.matmul(rot,e3)

        # Desired Attitude and Angular Velocity
        yaw_des = qd[qn]["yaw_des"]
        yawdot_des = qd[qn]["yawdot_des"]
        Rot_des = np.zeros((3,3), dtype=float)
        Z_body_in_world = Force/np.linalg.norm(Force)
        Rot_des[:, 2:3] = Z_body_in_world
        Y_unit = np.array([[-np.sin(yaw_des)], [np.cos(yaw_des)], [0]])
        X_body_in_world = np.cross(Y_unit, Z_body_in_world, axisa=0, axisb=0).T
        X_body_in_world = X_body_in_world/np.linalg.norm(X_body_in_world)
        Rot_des[:,0:1] = X_body_in_world
        Y_body_in_world = np.cross(Z_body_in_world, X_body_in_world, axisa=0, axisb=0).T
        Y_body_in_world = Y_body_in_world/np.linalg.norm(Y_body_in_world)
        Rot_des[:,1:2] = Y_body_in_world

        #print(Rot_des)
        #print(X_body_in_world)
        #print(Y_body_in_world)
        #print(Z_body_in_world)
        # p_des = -(m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*Y_body_in_world;
        # q_des = (m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*X_body_in_world;
        p_des = np.array([[0.0]])
        q_des = np.array([[0.0]])
        r_des = yawdot_des*Z_body_in_world[2:3, :]
        qd[qn]["rot_des"] = Rot_des
        qd[qn]["omega_des"] = np.vstack((p_des, q_des, r_des))

        # Quadrotor Attitude Control
        M = self.quadrotor_attitude_controller(qd[qn], params)

        return F, M, Rot_des

    # test passed
    def quadrotor_attitude_controller(self, qd, params):
        Rot = qd["rot"]
        Rot_des = qd["rot_des"]
        omega_des = qd["omega_des"]

        # errors of angles and angular velocities
        #e_Rot = np.matmul(np.transpose(Rot_des), Rot) - np.matmul(np.transpose(Rot), Rot_des)
        e_Rot = np.matmul(Rot_des.T, Rot) - np.matmul(Rot.T, Rot_des)
        e_angle = vee(e_Rot)/2
        '''print("The current rotation is")
        print(Rot)
        print("The des rotation is")
        print(Rot_des)
        print("The rotation error")
        print(e_angle)'''

        e_omega = qd["omega"] - np.matmul(Rot.T, np.matmul(Rot_des, omega_des))
        # e_omega = qd["omega"] - np.matmul(np.matmul(np.transpose(Rot), Rot_des), omega_des)
        # moment
        M = np.cross(qd["omega"], np.matmul(params.I, qd["omega"]), axisa=0, axisb=0).T - np.matmul(params.Kpe, e_angle) - np.matmul(params.Kde, e_omega) 
        #print("The e_angle", e_angle)
        #print("The @ multiply", params.Kpe @ e_angle)
        #print("The matmul multiply", np.matmul(params.Kpe, e_angle))
        return M

    # test passed
    def cooperative_suspended_payload_controller(self, ql, qd, pl_params, qd_params):

        if not pl_params.sim_start:
            # self.coeff0 = pl_params.coeff0
            self.icnt = 0
        self.icnt = self.icnt + 1

        # Parameter Initialization
        quat_des = ql["quat_des"]
        omega_des = ql["omega_des"]
        g = pl_params.grav
        m = pl_params.mass
        nquad = pl_params.nquad

        e3 = np.array([[0],[0],[1.0]])

        Rot = ql["rot"]
        omega_asym = vec2asym(ql["omega"])
        Rot_des = utilslib.QuatToRot(quat_des)

        ## Position control
        #  Position error
        ep = ql["pos_des"]-ql["pos"]
        # Velocity error
        ed = ql["vel_des"]-ql["vel"]

        # Desired acceleration This equation drives the errors of trajectory to zero.
        acceleration_des = ql["acc_des"] + np.matmul(pl_params.Kp, ep) + np.matmul(pl_params.Kd, ed)

        # Net force F=kx*ex kv*ex_dot + mge3 +mxdes_ddot
        F = m*g*e3  + m*acceleration_des

        ## Attitude Control
        # Errors of anlges and angular velocities
        e_Rot = Rot_des.T @ Rot - Rot.T @ Rot_des
        e_angle = np.divide(vee(e_Rot), 2)
        e_omega = ql["omega"] - Rot.T @ Rot_des @ omega_des.T 
        # e_omega = ql["omega"] - np.matmul(np.matmul(np.transpose(Rot), Rot_des), np.transpose(omega_des))

        # Net moment
        # Missing the angular acceleration term but in general it is neglectable.
        M = np.matmul(-pl_params.Kpe, e_angle) - np.matmul(pl_params.Kde, e_omega) # may need to be changed to scalar product

        # Cable tension distribution
        diag_rot = np.zeros((0,0), dtype=float)
        for i in range(1, nquad+1):
            diag_rot = LA.block_diag(diag_rot, Rot)

        mu = diag_rot @ pl_params.pseudo_inv_P @ np.append(Rot.T @ F, M, axis=0)
        for i in range(1, nquad+1):
            if (0>mu[3*i-1, 0]):
                mu[3*i-1, 0] = 0 
                print("mu is less than zero")
            else:# Is this really necessary? 
                mu[3*i-1, 0] = mu[3*i-1, 0]

        att_acc_c = acceleration_des + g*e3 + np.matmul(np.matmul(np.matmul(Rot, omega_asym), omega_asym), pl_params.rho_vec_list)

        # Quadrotor Attitude Controller
        qd_F = {}
        qd_M = {}
        qd_rot_des = {}
        qd_quat_des = {}

        for qn in range(0, nquad):
            qd[qn]["yaw_des"] = 0
            qd[qn]["yawdot_des"] = 0
            qd[qn]["mu_des"] = mu[3*qn:3*(qn+1)]
            qd[qn]["attach_accel"] = att_acc_c[:,qn].reshape((3,1))
            [F_qn, M_qn, Rot_des_qn] = self.cooperative_attitude_controller(qd, qn, qd_params)
            qd_F[qn] = F_qn
            qd_M[qn] = M_qn
            qd_quat_des[qn] = tranrot.from_matrix(Rot_des_qn).as_quat() 
            qd_rot_des[qn] = Rot_des_qn 
    
        #return qd_F, qd_M
        return mu, att_acc_c, qd_F, qd_M, qd_quat_des, qd_rot_des

    # untested
    def cooperative_payload_controller(self, ql, params):
        if not params["sim_start"]:
            # self.coeff0 = params.coeff0
            self.icnt = 0
        self.icnt = self.icnt + 1

        ## Parameter Initialization
        quat_des = ql["quat_des"]
        omega_des = ql["omega_des"]
        g = params.grav
        m = params.mass

        e3 = np.array([[0],[0],[1]])

        Rot = ql["rot"]
        omega_asym = vec2asym(ql["omega"])
        Rot_des = utilslib.QuatToRot(quat_des)

        ## Position control
        # jerk_des = ql.jerk_des;
        # Position error
        ep = ql["pos_des"]-ql["pos"]
        # Velocity error
        ed = ql["vel_des"]-ql["vel"]

        # Desired acceleration This equation drives the errors of trajectory to zero.
        acceleration_des = ql["acc_des"] + np.matmul(params.Kp, ep) + np.matmul(params.Kd, ed)

        # Net force F=kx*ex kv*ex_dot + mge3 +mxdes_ddot
        F = m*g*e3  + m*acceleration_des

        ## Attitude Control
        # Errors of anlges and angular velocities
        e_Rot = np.matmul(np.transpose(Rot_des), Rot) - np.matmul(np.transpose(Rot), Rot_des)
        e_angle = vee(e_Rot)/2
        e_omega = ql["omega"] - np.matmul(np.matmul(np.transpose(Rot), Rot_des), np.transpose(omega_des))

        # Net moment
        # Missing the angular acceleration term but in general it is neglectable.
        M = - np.matmul(params.Kpe, e_angle) - np.matmul(params.Kde, e_omega)

        ## Cable tension distribution
        diag_rot = np.array([[]])
        for i in range(1, params.nquad+1):
            diag_rot = scipy.linalg.block_diag(diag_rot,Rot)

        mu = np.matmul(np.matmul(diag_rot, params.pseudo_inv_P), np.vstack(np.matmul(np.transpose(Rot), F), M))
        for i in range(1, params.nquad+1):
            if mu[3*i-1]<0:
                mu[3*i-1] = 0
        

        att_acc_c = acceleration_des + g @ e3 + Rot @ omega_asym @ omega_asym @ params.rho_vec_list

        return mu,att_acc_c

    # untested
    def geometric_controller(self, qd, t, qn, params):
        if self.gd.size == 0:
            self.gd = np.zeros((0,3), dtype= float)
            self.icnt = 0
        self.icnt += 1

        ## Parameter Initialization
        yaw_des = qd[qn]["yaw_des"]
        yawdot_des = qd[qn]["yawdot_des"]
        g = params.grav
        m = params.mass
        phi = qd[qn]["euler"][0]
        theta = qd[qn]["euler"][1]
        psi = qd[qn]["euler"][2]
        e3 = np.array([[0], [0], [1]])

        #   The rotation matrix in this function is world to body [bRw] you will
        #   need to transpose this matrix to get the body to world [wRb] such that
        #   [wP] = [wRb] * [bP], where [bP] is a point in the body frame and [wP]
        #   is a point in the world frame
        Rot_worldtobody = RPYtoRot_ZXY(phi, theta, psi)

        ## Position control 
        jerk_des = qd[qn]["jerk_des"]
        # Position error
        ep = qd[qn]["pos_des"]-qd[qn]["pos"]
        # Velocity error
        ed = qd[qn]["vel_des"]-qd[qn]["vel"]

        # Desired acceleration This equation drives the errors of trajectory to zero.
        acceleration_des = qd[qn]["acc_des"] + params.Kp @ ep + params.Kd @ ed;  

        # Thurst f=(kx*ex kv*ex_dot + mge3 +mxdes_ddot)*Re3
        Force = m*g*e3  + m*acceleration_des
        F = np.transpose(Force) @ np.transpose(Rot_worldtobody) @ e3

        ## Attitude Control 
        Rot_des = np.zeros((3,3), dtype=float)
        Z_body_in_world = Force/np.linalg.norm(Force)
        Rot_des[:,2] = Z_body_in_world
        X_unit = np.vstack(np.cos(yaw_des), np.sin(yaw_des), 0)
        Y_body_in_world = np.cross(Z_body_in_world,X_unit)
        Y_body_in_world = Y_body_in_world/np.linalg.norm(Y_body_in_world)
        Rot_des[:,1] = Y_body_in_world
        X_body_in_world = np.cross(Y_body_in_world,Z_body_in_world)
        Rot_des[:,0] = X_body_in_world

        # Errors of anlges and angular velocities
        e_Rot = np.transpose(Rot_des) @ np.transpose(Rot_worldtobody) - Rot_worldtobody @ Rot_des
        e_angle = vee(e_Rot)/2

        p_des = -(m/F) * np.transpose(jerk_des - (np.transpose(Z_body_in_world) @ jerk_des) @ Z_body_in_world) @ Y_body_in_world
        q_des = (m/F) * np.transpose(jerk_des - (np.transpose(Z_body_in_world) @ jerk_des) @ Z_body_in_world) @ X_body_in_world
        r_des = yawdot_des * Z_body_in_world[2]
        e_omega = qd[qn]["omega"] - Rot_worldtobody @ Rot_des @ np.transpose(np.hstack(p_des, q_des, r_des))

        # Moment
        # Missing the angular acceleration term but in general it is neglectable.
        M = - params.Kpe @ e_angle - params.Kde @ e_omega + np.cross(qd[qn]["omega"], params.I*qd[qn]["omega"])

        # =================== Your code ends here ===================

        # Output trpy and drpy as in hardware
        trpy = np.array([0,0,0,0])
        drpy = np.array([0,0,0,0])
        return F, M, trpy, drpy
    
    # untested
    def hover_controller(self, qd, t, qn, params):
        if self.gd.size == 0:
            self.gd = np.zeros((0,3), dtype= float)
            self.icnt = 0
        self.icnt += 1

        # position_now = qd{qn}.pos;
        # velocity_now = qd{qn}.vel;
        # Eulerangle_now = qd{qn}.euler;
        # omega_now = qd{qn}.omega;
        # position_tra = qd{qn}.pos_des;
        # velocity_tra = qd{qn}.vel_des;
        # acceleration_tra = qd{qn}.acc_des;

        ## Parameter Initialization
        yaw_des = qd[qn]["yaw_des"]
        yawdot_des = qd[qn]["yawdot_des"]
        g = params.grav
        m = params.mass
        # Gain matrices
        Kp_pos = np.array([[5, 0, 0],
                           [0, 5, 0],
                           [0, 0, 150]])
        Kp_att = np.array([[5, 0, 0],
                           [0, 5, 0],
                           [0, 0, 150]])
        Kd_att = np.array([[5.5, 0, 0],
                           [0, 5.5, 0],
                           [0, 0, 150]])
        Ki_att = np.array([[0.004, 0, 0],
                           [0, 0.004, 0],
                           [0, 0, 0.004]])
        Kpe    = np.array([[0.1, 0, 0],
                           [0, 0.1, 0],
                           [0, 0, 0.2]])
        Kde    = np.array([[0.004, 0, 0],
                           [0, 0.004, 0],
                           [0, 0, 0.004]])

        ## Position control
        # Position error
        e_pos = qd[qn]["pos_des"]-qd[qn]["pos"]
        vel_des = Kp_pos @ e_pos
        # Velocity error
        e_vel = vel_des-qd[qn]["vel"]

        ## Hover controller
        # Desired acceleration This equation drives the errors of trajectory to zero.
        acceleration_des = qd[qn]["acc_des"] + params.Kp @ e_pos + params.Kd @ e_vel

        #  Desired roll, pitch and yaw
        phi_des = (acceleration_des[0]*np.sin(yaw_des)-acceleration_des[1]*np.cos(yaw_des))/g
        theta_des = (acceleration_des[0]*np.cos(yaw_des)+acceleration_des[1]*np.sin(yaw_des))/g
        psi_des = yaw_des

        # Errors of anlges and angular velocities
        e_angle = np.transpose(np.hstack(phi_des, theta_des, psi_des)) - qd[qn]["euler"]
        e_omega = np.transpose(np.hstack(0, 0, yawdot_des)) - qd[qn]["omega"]

        # Thurst
        F = m*g  + m*acceleration_des[2]

        # Moment
        M = Kpe @ e_angle + Kde @ e_omega

        # 
        self.gd[self.icnt-1,:] = np.hstack(t, phi_des, qd[qn]["euler"][0])  # for graphing

        # Output trpy and drpy as in hardware
        trpy = np.array([0,0,0,0])
        drpy = np.array([0,0,0,0])
        return F, M, trpy, drpy

    # untested
    def rigid_links_cooperative_payload_controller(self, ql, params):
        if not params["sim_start"]:
            self.icnt = 0
        self.icnt = self.icnt + 1

        ## Parameter Initialization
        quat_des = ql["quat_des"]
        yaw_des = 0
        omega_des = ql["omega_des"]
        g = params.grav
        m = params.struct_mass

        e3 = np.array([[0],[0],[1]])

        Rot = ql["rot"]
        omega = ql["omega"]

        ## Position control
        # jerk_des = ql.jerk_des;
        # Position error
        ep = ql["pos_des"]-ql["pos"]
        # Velocity error
        ed = ql["vel_des"]-ql["vel"]

        # Desired acceleration This equation drives the errors of trajectory to zero.
        acceleration_des = ql["acc_des"] + params.Kp @ ep + params.Kd @ ed

        # Net force F=kx*ex kv*ex_dot + mge3 +mxdes_ddot
        Force = m*g*e3  + m*acceleration_des
        tau = np.transpose(Force) @ Rot @ e3

        Rot_des = np.zeros((3,3), dtype=float)
        Z_body_in_world = Force/np.linalg.norm(Force)
        Rot_des[:,2] = Z_body_in_world
        X_unit = np.vstack(np.cos(yaw_des), np.sin(yaw_des), 0)
        Y_body_in_world = np.cross(Z_body_in_world,X_unit)
        Y_body_in_world = Y_body_in_world/np.linalg.norm(Y_body_in_world)
        Rot_des[:,1] = Y_body_in_world
        X_body_in_world = np.cross(Y_body_in_world,Z_body_in_world)
        Rot_des[:,0] = X_body_in_world

        ## Attitude Control

        # Errors of anlges and angular velocities
        e_Rot = np.transpose(Rot_des) @ Rot - np.transpose(Rot) @ Rot_des
        e_angle = vee(e_Rot)/2
        e_omega = omega - np.transpose(Rot) @ Rot_des @ np.transpose(omega_des)

        # Net moment
        # Missing the angular acceleration term but in general it is neglectable.
        M = - params.Kpe @ e_angle - params.Kde @ e_omega + np.cross(omega, params.struct_I @ omega)

        ## Quadrotor Thrust and Moment Distribution
        u = params["thrust_moment_distribution_mat"] @ np.vstack(tau. M)

        return u

    # tested
    def single_payload_geometric_controller(self, ql, qd_params, pl_params):
        ## Parameter Initialization
        if not pl_params.sim_start:
            self.icnt = 0
            g = pl_params.grav
            e3 = np.array([[0],[0],[1]])

        self.icnt = self.icnt + 1
        quad_m = qd_params.mass
        pl_m = pl_params.mass
        l = pl_params.cable_length


        ## State Initialization
        quad_load_rel_pos = ql["qd_pos"]-ql["pos"]
        quad_load_rel_vel = ql["qd_vel"]-ql["vel"]
        quad_load_distance = np.linalg.norm(quad_load_rel_pos)
        xi_ = -quad_load_rel_pos/quad_load_distance
        xixiT_ = xi_ @ np.transpose(xi_)
        xidot_ = -quad_load_rel_vel/quad_load_distance
        xi_asym_ = vec2asym(xi_)
        w_ = np.cross(xi_, xidot_, axisa=0, axisb=0).T
        Rot_worldtobody = np.transpose(ql["qd_rot"])

        ## Payload Position control
        #jerk_des = ql.jerk_des;
        #Position error
        ep = ql["pos_des"]-ql["pos"]
        #Velocity error
        ed = ql["vel_des"]-ql["vel"]

        # Desired acceleration This equation drives the errors of trajectory to zero.
        acceleration_des = ql["acc_des"] + g*e3 + pl_params.Kp @ ep + pl_params.Kd @ ed

        # Desired yaw and yawdot
        yaw_des = ql["qd_yaw_des"] # This can remain for Quad
        yawdot_des = ql["qd_yawdot_des"]

        ## Cable Direction Control
        # Desired cable direction

        mu_des_ = (quad_m + pl_m) * acceleration_des + quad_m * l * (np.transpose(xidot_) @ xidot_) * xi_
        xi_des_ = -mu_des_ / np.linalg.norm(mu_des_)
        xi_des_dot_ = np.zeros((3, 1), dtype=float)     #  TODO: design q_des_dot_
        w_des_ = np.cross(xi_des_, xi_des_dot_, axisa=0, axisb=0).T
        w_des_dot_ = np.zeros((3, 1), dtype=float)      # TODO: design w_des_dot_
        mu_ = xixiT_ @ mu_des_

        e_xi = np.cross(xi_des_, xi_, axisa=0, axisb=0).T
        e_w = w_ + xi_asym_ @ xi_asym_ @ w_des_
        Force = mu_ - quad_m*l*np.cross(xi_, qd_params.Kxi @ e_xi + qd_params.Kw @ e_w+ (xi_.T @ w_des_) * xidot_ + xi_asym_ @ xi_asym_ @ w_des_dot_, axisa=0, axisb=0).T
        F = np.transpose(Force) @ np.transpose(Rot_worldtobody) @ e3

        ## Attitude Control
        Rot_des = np.zeros((3,3), dtype=float)
        Z_body_in_world = Force/np.linalg.norm(Force)
        Rot_des[:,2:3] = Z_body_in_world
        Y_unit = np.array([[-np.sin(yaw_des)], [np.cos(yaw_des)], [0]])
        X_body_in_world = np.cross(Y_unit, Z_body_in_world, axisa=0, axisb=0).T
        X_body_in_world = X_body_in_world/np.linalg.norm(X_body_in_world)
        Rot_des[:,0:1] = X_body_in_world
        Y_body_in_world = np.cross(Z_body_in_world, X_body_in_world, axisa=0, axisb=0).T
        Y_body_in_world = Y_body_in_world/np.linalg.norm(Y_body_in_world)
        Rot_des[:,1:2] = Y_body_in_world

        '''
        Rot_des = np.zeros((3,3), dtype=float)
        Z_body_in_world = Force/np.linalg.norm(Force)
        Rot_des[:,2:3] = Z_body_in_world
        X_unit = np.array([[np.cos(yaw_des)], [np.sin(yaw_des)], [0]])
        Y_body_in_world = np.cross(Z_body_in_world, X_unit, axisa=0, axisb=0).T
        Y_body_in_world = Y_body_in_world/np.linalg.norm(Y_body_in_world)
        Rot_des[:,1:2] = Y_body_in_world
        X_body_in_world = np.cross(Y_body_in_world,Z_body_in_world, axisa=0, axisb=0).T
        Rot_des[:,0:1] = X_body_in_world'''

        '''
        Rot_des = np.zeros((3,3), dtype=float)
        Z_body_in_world = Force/np.linalg.norm(Force)
        Rot_des[:, 2:3] = Z_body_in_world
        Y_unit = np.array([[-np.sin(yaw_des)], [np.cos(yaw_des)], [0]])
        X_body_in_world = np.cross(Y_unit, Z_body_in_world, axisa=0, axisb=0).T
        X_body_in_world = X_body_in_world/np.linalg.norm(X_body_in_world)
        Rot_des[:,0:1] = X_body_in_world
        Y_body_in_world = np.cross(Z_body_in_world, X_body_in_world, axisa=0, axisb=0).T
        Y_body_in_world = Y_body_in_world/np.linalg.norm(Y_body_in_world)
        Rot_des[:,1:2] = Y_body_in_world'''

        # Errors of anlges and angular velocities
        e_Rot = np.transpose(Rot_des) @ np.transpose(Rot_worldtobody) - Rot_worldtobody @ Rot_des
        e_angle = vee(e_Rot)/2

        # TODO: implement p_des and q_des using differentially flat property
        # p_des = -(m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*Y_body_in_world;
        # q_des = (m/F)*(jrk_des - (Z_body_in_world'*jrk_des)*Z_body_in_world)'*X_body_in_world;
        p_des = 0.0
        q_des = 0.0
        r_des = yawdot_des*Z_body_in_world[2]
        e_omega = ql["qd_omega"] - Rot_worldtobody @ Rot_des @ np.array([[p_des], [q_des], [r_des]])

        # Moment
        # Missing the angular acceleration term but in general it is neglectable.
        M = - qd_params.Kpe @ e_angle - qd_params.Kde @ e_omega + np.cross(ql["qd_omega"],qd_params.I @ ql["qd_omega"], axisa=0, axisb=0).T
        return F, M
