import numpy as np
    
def cooperativeGuard(t = None,x = None,pl_params = None,slack_condition = None): 
    # slackToTaut function event function for the integration
    
    # INPUTS:
# t             - 1 x 1, time
# x             - (13 + 13*nquad) x 1,
#                 state vector = [xL, yL, zL, xLd, yLd, zLd,
#                                 qLw, qLx, qLy, qLz, pL, qL, rL,
#                                 [xQ, yQ, zQ, xQd, yQd, zQd]_i, i = 1,...,nquad
#                                 [qw, qx, qy, qz, pQ, qQ, rQ]_i, i = 1,...,nquad
# nquad         - number of quads
# cable_length  - The cable's length
    
    # OUTPUTS:
# value         - (attach points - robot distance) - cable_length
# isterminal    - boolean flag representing stop the integration
# direction     - approaching direction of the value
    
    # find the idx of the cables that are slack
    idx = np.arange(1,pl_params.nquad+1)
    slack_cable_idx = idx(slack_condition == 1)
    taut_cable_idx = idx(slack_condition == 0)
    num_of_slack_cable = len(slack_cable_idx)
    num_of_taut_cable = len(taut_cable_idx)
    # The rotation matrix of the payload
    RotL = np.transpose(QuatToRot(x(np.arange(7,10+1))))
    # The attach points' positions correspond to the slack cables.
    attach_pts = x(np.arange(1,3+1)) + RotL * pl_params.rho_vec_list
    # The quadrotor positions correspond to the slack cables.
    slack_quad_pos_idx = 13 * slack_cable_idx + np.array([[1],[2],[3]])
    taut_quad_pos_idx = 13 * taut_cable_idx + np.array([[1],[2],[3]])
    # Set up the condition to terminate the integration.
# Detect cable-robot distance = 0
    value = np.transpose(np.array([vecnorm(x(slack_quad_pos_idx) - attach_pts(:,slack_cable_idx),2,1) - pl_params.cable_length(slack_cable_idx),vecnorm(x(taut_quad_pos_idx) - attach_pts(:,taut_cable_idx),2,1) - pl_params.cable_length(taut_cable_idx) + 0.0001]))
    isterminal = np.ones((pl_params.nquad,1))
    
    direction = np.array([[np.ones((num_of_slack_cable,1))],[- np.ones((num_of_taut_cable,1))]])
    
    return value,isterminal,direction
    
    return value,isterminal,direction