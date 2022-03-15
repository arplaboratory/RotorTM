import numpy as np
    
def rigidbody_payloadEOM_readonly(t = None,s = None,F = None,M = None,params = None): 
    # QUADEOM_READONLY Solve quadrotor equation of motion
#   quadEOM_readonly calculate the derivative of the state vector
    
    # INPUTS:
# t      - 1 x 1, time
# s      - 13 x 1, state vector = [xL, yL, zL, xLd, yLd, zLd, qw, qx, qy, qz, pL,qL,rL]
# F      - 1 x 1, thrust output from controller (only used in simulation)
# M      - 3 x 1, moments output from controller (only used in simulation)
# params - struct, output from crazyflie() and whatever parameters you want to pass in
    
    # OUTPUTS:
# sdot   - 13 x 1, derivative of state vector s
    
    # NOTE: You should not modify this function
# See Also: quadEOM_readonly, crazyflie
    
    #************ EQUATIONS OF MOTION ************************
# Limit the force and moments due to actuator limits
    
    # Assign states
    quat = s.quat
    qW = quat(1)
    qX = quat(2)
    qY = quat(3)
    qZ = quat(4)
    omega = s.omega
    p = omega(1)
    q = omega(2)
    r = omega(3)
    # Payload quaternion first derivative
    K_quat = 2
    
    quaterror = 1 - (qW ** 2 + qX ** 2 + qY ** 2 + qZ ** 2)
    qLdot = - 1 / 2 * np.array([[0,- p,- q,- r],[p,0,- r,q],[q,r,0,- p],[r,- q,p,0]]) * quat + K_quat * quaterror * quat
    # Payload Angular acceleration
    effective_M = M - s.C * s.invML * F - cross(s.omega,params.I * s.omega)
    effective_inertia = params.I + s.C * s.invML * s.D - s.E
    omgLdot = np.linalg.solve(effective_inertia,effective_M)
    # Payload Acceleration
    accL = s.invML * (F + s.D * omgLdot) - params.grav * np.array([[0],[0],[1]])
    # Assemble sdot
    sdotLoad = np.zeros((13,1))
    sdotLoad[np.arange[1,3+1]] = s.vel
    sdotLoad[4] = accL(1)
    sdotLoad[5] = accL(2)
    sdotLoad[6] = accL(3)
    sdotLoad[7] = qLdot(1)
    sdotLoad[8] = qLdot(2)
    sdotLoad[9] = qLdot(3)
    sdotLoad[10] = qLdot(4)
    sdotLoad[11] = omgLdot(1)
    sdotLoad[12] = omgLdot(2)
    sdotLoad[13] = omgLdot(3)
    return sdotLoad
    
    return sdotLoad