import numpy as np
    
def tautToSlack(t = None,x = None,cable_length = None): 
    # slackToTaut function event function for the integration
    
    # INPUTS:
# t             - 1 x 1, time
# x             - 19 x 1,
#                 state vector = [xL, yL, zL, xLd, yLd, zLd,
#                                 qLw, qLx, qLy, qLz, pL, qL, rL,
#                                 xQ, yQ, zQ, xQd, yQd, zQd,
#                                 qw, qx, qy, qz, pQ, qQ, rQ]
# nquad         - number of quads
# cable_length  - The cable's length
    
    # OUTPUTS:
# value         - (cable-robot distance) - cable_length
# isterminal    - boolean flag representing stop the integration
# direction     - approaching direction of the value
    
    value = norm(x(np.arange(1,3+1)) - x(np.arange(7,9+1))) - cable_length + 0.001
    
    isterminal = 1
    
    direction = - 1
    
    return value,isterminal,direction
    
    return value,isterminal,direction