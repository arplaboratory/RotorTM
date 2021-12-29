import numpy as np
    
def robot_team_setup(payload_params_path = None,quad_params_path = None,mechanism_params_path = None): 
    ## robot_team_setup: This function setup the robot teams parameters
    
    # 2021 Guanrui Li
    
    # This function creates a struct with the basic parameters for the
# triangular payload
    
    # Model assumptions based on physical measurements:
    
    # You can add any fields you want in params
    
    payload_params = read_payload_params(payload_params_path)
    quad_params = read_uav_params(quad_params_path)
    mechanism_params = read_mechanism_params(mechanism_params_path)
    params = payload_params
    ## **************** Read attach mechanism parameters ******************
    
    params.nquad = mechanism_params.nquad
    params.mechanism_type = mechanism_params.type
    params.rho_vec_list = mechanism_params.rho_vec_list
    rho_vec_asym_mat = []
    identity_stack_mat = []
    for k in np.arange(1,params.nquad+1).reshape(-1):
        rho_vec_asym_mat = np.array([rho_vec_asym_mat,vec2asym(mechanism_params.rho_vec_list(:,k))])
        identity_stack_mat = np.array([identity_stack_mat,np.eye(3)])
    
    params.rho_vec_asym_mat = rho_vec_asym_mat
    params.vertices = np.array([[np.transpose(mechanism_params.rho_vec_list)],[np.transpose((mechanism_params.rho_vec_list - np.array([[0],[0],[0.1]])))]])
    ## Set up parameters
    if mechanism_params.type == 'Cable':
        ## This section sets up the essential controller parameters for cable suspended payload
        params.cable_length = mechanism_params.cable_length
        if params.payload_type == 'Rigid Body':
            # Geometric parameters matrix for cooperative geometric controller
            params.P = np.array([[identity_stack_mat],[rho_vec_asym_mat]])
            params.P_bar = null(params.P)
            __,nullspace_dim = params.P_bar.shape
            params.coeff0 = np.zeros((nullspace_dim,1))
            params.nullspace_dim = nullspace_dim
            params.pseudo_inv_P = np.transpose(params.P) * inv(params.P * np.transpose(params.P))
        params.robot_radius = 0.3
    else:
        if mechanism_params.type == 'Rigid Link':
            # This section sets up the essential controller parameters for payload with rigid links
## Physical properties of structure
            # Calculate the inertia and mass of the entire structure
            params.struct_I = params.I
            params.struct_mass = params.mass
            rho_c = np.zeros((3,1))
            for k in np.arange(1,params.nquad+1).reshape(-1):
                params.struct_mass = params.struct_mass + quad_params.mass
                rho_c = rho_c + quad_params.mass * mechanism_params.rho_vec_list(:,k)
            rho_c = rho_c / (quad_params.mass * params.nquad + params.mass)
            params.struct_I = params.struct_I + params.mass * np.array([[rho_c(2) ** 2,0,0],[0,rho_c(1) ** 2,0],[0,0,rho_c(1) ** 2 + rho_c(2) ** 2]])
            # Calculate the geometric constraints of the structure
            A = []
            for k in np.arange(1,params.nquad+1).reshape(-1):
                rho = mechanism_params.rho_vec_list(:,k) - rho_c
                R = np.transpose(RPYToRot_ZXY(0,0,mechanism_params.yaw_list(k)))
                params.struct_I = params.struct_I + R * quad_params.I * np.transpose(R) + quad_params.mass * np.array([[rho(2) ** 2,0,0],[0,rho(1) ** 2,0],[0,0,rho(1) ** 2 + rho(2) ** 2]])
                A = np.array([A,np.array([[1,0,0,0],[np.array([[rho(2)],[rho(1)],[0]]),R]])])
            params.rho_load = - rho_c
            params.rho_robot = mechanism_params.rho_vec_list - rho_c
            params.A = A
            # Distribution matrix
            W = []
            for k in np.arange(1,params.nquad+1).reshape(-1):
                W = blkdiag(W,np.array([[1,np.zeros((1,3))],[np.zeros((3,1)),10 * np.eye(3)]]))
            invW = inv(W)
            params.thrust_moment_distribution_mat = invW * np.transpose(A) * inv(A * invW * np.transpose(A))
    
    return params
    
    return params