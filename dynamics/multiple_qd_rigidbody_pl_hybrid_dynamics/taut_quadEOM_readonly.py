import numpy as np
    
def taut_quadEOM_readonly(t = None,qd = None,params = None): 
    # QUADEOM_READONLY Solve quadrotor equation of motion when the cable is
# slack.
# quadEOM_readonly calculate the derivative of the state vector
    
    # INPUTS:
# t      - 1 x 1, time
# qd     - struct, quadrotor states and inputs
# params - struct, parameters you want to pass in
    
    # OUTPUTS:
# sdot   - 13 x 1, derivative of state vector s
    
    #************ EQUATIONS OF MOTION ************************
    
    # Assign states
    F = qd.F
    M = qd.M
    T = qd.T
    xi = qd.xi
    vel = qd.vel
    quat = qd.quat
    omega = qd.omega
    qW = quat(1)
    qX = quat(2)
    qY = quat(3)
    qZ = quat(4)
    p = omega(1)
    q = omega(2)
    r = omega(3)
    bRw = QuatToRot(quat)
    wRb = np.transpose(bRw)
    # Acceleration
    accel = 1 / params.mass * (wRb * np.array([[0],[0],[F]]) + T * xi - np.array([[0],[0],[params.mass * params.grav]]))
    
    # Angular velocity
    K_quat = 2
    
    quaterror = 1 - (qW ** 2 + qX ** 2 + qY ** 2 + qZ ** 2)
    qdot = - 1 / 2 * np.array([[0,- p,- q,- r],[p,0,- r,q],[q,r,0,- p],[r,- q,p,0]]) * quat + K_quat * quaterror * quat
    # Angular acceleration
    pqrdot = params.invI * (M - cross(omega,params.I * omega))
    # Assemble sdot
    sdot = np.zeros((13,1))
    sdot[1] = vel(1)
    sdot[2] = vel(2)
    sdot[3] = vel(3)
    sdot[4] = accel(1)
    sdot[5] = accel(2)
    sdot[6] = accel(3)
    sdot[7] = qdot(1)
    sdot[8] = qdot(2)
    sdot[9] = qdot(3)
    sdot[10] = qdot(4)
    sdot[11] = pqrdot(1)
    sdot[12] = pqrdot(2)
    sdot[13] = pqrdot(3)
    return sdot
    
    return sdot