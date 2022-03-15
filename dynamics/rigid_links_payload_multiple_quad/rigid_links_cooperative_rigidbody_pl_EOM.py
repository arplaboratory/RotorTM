import numpy as np
    
def rigid_links_cooperative_rigidbody_pl_EOM(t = None,s = None,nquad = None,plcontrolhdl = None,trajhandle = None,quad_params = None,pl_params = None): 
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
    
    # convert state to payload stuct for control
    structure_state = s
    structure = stateToPl(structure_state)
    # Get desired_state for the payload
    
    structure = trajhandle(t,structure)
    
    ## Payload Controller (Distribution of Cables' Direction and Tension)
    robot_thrust_moment = plcontrolhdl(structure,pl_params)
    ## Combine robot's forces and moments to net force and moment
    u = pl_params.A * robot_thrust_moment
    F = u(1) * structure.rot(:,3)
    M = u(np.arange(2,4+1))
    ## Dynamics of Structure
    sdot = rigidbody_structEOM_readonly(t,structure,F,M,pl_params)
    return sdot
    
    return sdot