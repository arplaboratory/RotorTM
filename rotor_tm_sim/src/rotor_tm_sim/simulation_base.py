import numpy as np
import numpy.linalg as linalg
import rospy
import scipy.integrate
from scipy.spatial.transform import Rotation as rot_math
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Wrench
from sensor_msgs.msg import Imu
from rotor_tm_msgs.msg import RPMCommand, FMCommand
from quadrotor_msgs.msg import ukf_measurement_update
from rotor_tm_utils import utilslib, rosutilslib
from rotor_tm_utils.vee import vee
import time
import scipy.linalg as LA

def ptmassslackToTaut(t, x):
    # DESCRIPTION:
    # event function for point mass scenario dynammics 
    # if event is reached by ivp solver, it will switch from slack to taut, takes in t (time) and state (x)

    # INPUTS:
    # t             - time
    # x             - state of the point mass system. Specifically,
    #                 x is a 26 by 1 ndarray,
    #                                                               Name                 Last Element Location (counting from 1) 
    #                 x = np.array([ppx,  ppy,    ppz,            # payload position    3
    #                               pvx,  pvy,    pvz,            # payload velocity    6
    #                                pu,   pi,     pj,     pk,    # payload quat        10
    #                               pwx,  pwy,    pwz,            # payload omega       13
    #                               qpx,  qpy,    qpz,            # quad rotor position 16
    #                               qvx,  qvy,    qvz,            # quad rotor velocity 19 
    #                                qu,   qi,     qj,     qk,    # quad rotor quat     23
    #                               qwx,  qwy,    qwz])           # quad rotor omega    26

    # OUTPUTS:
    # value         - a float that determines taut condition
    value = np.linalg.norm(x[0:3] - x[13:16]) - ptmassslackToTaut.cable_length
    return value

def ptmasstautToSlack(t, x):
    # DESCRIPTION:
    # event function for point mass scenario dynammics 
    # if event is reached by ivp solver, it will switch from taut to slack, takes in t (time) and state (x)

    # INPUTS:
    # t             - time
    # x             - state of the point mass system. Specifically,
    #                 x is a 26 by 1 ndarray,
    #                                                               Name                 Last Element Location (counting from 1) 
    #                 x = np.array([ppx,  ppy,    ppz,            # payload position    3
    #                               pvx,  pvy,    pvz,            # payload velocity    6
    #                                pu,   pi,     pj,     pk,    # payload quat        10
    #                               pwx,  pwy,    pwz,            # payload omega       13
    #                               qpx,  qpy,    qpz,            # quad rotor position 16
    #                               qvx,  qvy,    qvz,            # quad rotor velocity 19 
    #                                qu,   qi,     qj,     qk,    # quad rotor quat     23
    #                               qwx,  qwy,    qwz])           # quad rotor omega    26

    # OUTPUTS:
    # value         - a float that determines slack condition
    value = np.linalg.norm(x[0:3] - x[13:16]) - ptmasstautToSlack.cable_length + 0.000001
    return value

def cooperativeGuard(t, x, nquad, slack_condition, rho_vec_list, cable_length, id):
    # DESCRIPTION:
    # Event function event function for cooperative dynamics
    # Each MAV in the simulation will have its own event function
    # This function will be called under a lambda handle
    
    # INPUTS:
    # t                 - time
    # x                 - (13 + 13*nquad) x 1,
    #                   state vector = [xL, yL, zL, xLd, yLd, zLd, 
    #                                 qLw, qLx, qLy, qLz, pL, qL, rL, 
    #                                 [xQ, yQ, zQ, xQd, yQd, zQd]_i, i = 1,...,nquad
    #                                 [qw, qx, qy, qz, pQ, qQ, rQ]_i, i = 1,...,nquad
    # nquad             - number of quads
    # slack_condition   - a size nquad array, denoting cable condition for each MAV
    # rho_vec_list      - a nquad by nquad matrix, denoting mounting position for each MAV
    # cable_length      - The cable's length
    # id                - a scalar, denoting the current MAV number

    # OUTPUTS:
    # value         - (attach points - robot distance) - cable_length

    # find the idx of the cables that are slack
    idx = np.arange(1, nquad+1)
    slack_cable_idx = idx[slack_condition == 1]
    taut_cable_idx = idx[slack_condition == 0]

    # The rotation matrix of the payload
    RotL = utilslib.QuatToRot(x[6:10])

    # The attach points' positions correspond to the slack cables. 
    attach_pts = x[0:3].reshape((3,1)) + RotL @ rho_vec_list

    # The quadrotor positions correspond to the slack cables. 
    slack_quad_pos_idx = 13*slack_cable_idx + np.array([[0],[1],[2]])
    taut_quad_pos_idx = 13*taut_cable_idx + np.array([[0],[1],[2]])

    # Set up the condition to terminate the integration.
    # Detect cable-robot distance = 0
    left = np.linalg.norm(x[slack_quad_pos_idx] - attach_pts[:,slack_cable_idx - 1],2,0)-cable_length[slack_cable_idx - 1]
    right = np.linalg.norm(x[taut_quad_pos_idx] - attach_pts[:,taut_cable_idx - 1],2,0)-cable_length[taut_cable_idx - 1] + 0.0001
    value = np.transpose(np.hstack((left, right)))
    return value[id]

class simulation_base():
  def __init__(self,pl_params,uav_params):
      rospy.init_node('simulation')
      self.rate = 100
      white_noise_cov = np.diag(np.zeros(12))
      uav_white_noise_cov = np.diag(np.zeros(12))
      #white_noise_cov = np.diag(np.array([1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4]))
      #uav_white_noise_cov = np.diag(np.array([1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4,1e-4]))
      #white_noise_cov = np.diag(np.array([1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6]))
      #uav_white_noise_cov = np.diag(np.array([1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6,1e-6]))
      #white_noise_cov = np.diag(np.array([5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6]))
      #uav_white_noise_cov = np.diag(np.array([5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6,5e-6]))
      rate = rospy.Rate(self.rate)
      t_span = (0,1/self.rate)
      self.worldframe = "simulator"

      ################################## init parameters ################################
      self.pl_params = pl_params
      self.uav_params = uav_params
      self.nquad = self.pl_params.nquad
      self.pl_dim_num = 13
      self.uav_dim_num = 13
      self.mav_name = 'dragonfly'
      self.sim_start = False
      self.last_odom_time_received = 0.0
      self.ext_force = np.array([0.0,0.0,0.0])
      self.ext_torque = np.array([0.0,0.0,0.0])
      self.imu_accel = np.zeros((3, self.nquad), dtype=float)
      self.variance = 0.01**3   # variance for odom
      self.imu_variance = 0.01**2 # variance for imu
      self.wrench_value = Wrench()
      self.tension_ = np.zeros((3, self.nquad), dtype=float)
      # Three scenario:
      #                 1. Cooperative
      #                 2. Point mass
      #                 3. Rigid link
      # init parameters is done separately for each scenario below
      # with if-else branch structure

      if self.nquad != 1:
        # First Scenario: Cooperative  
        if self.pl_params.id == "Cable":
            print("Initalizing Cooperative Scenario")
            uav_hover_force = []
            for idx in range(self.nquad) : uav_hover_force.extend([0,0,self.uav_params[idx].mass * self.pl_params.grav]) 
            self.uav_F = np.matmul(self.pl_params.pseudo_inv_P, np.array([0,0,self.pl_params.mass * self.pl_params.grav,0,0,0])) + uav_hover_force
            self.uav_F = self.uav_F.reshape(self.nquad, 3)[:,2]
            self.uav_M = np.zeros((3,self.nquad))

            self.rho_vec_list = self.pl_params.rho_vec_list
            self.cable_len_list = np.array(self.pl_params.cable_length)
            x = np.zeros(self.pl_dim_num + self.uav_dim_num * self.nquad)
            for i in range(self.nquad+1): 
                if i > 0:
                    print("initalizing robot ", i)
                    x[i*13:i*13+3] = x[0:3] + self.rho_vec_list[:,i-1] + np.array([0,0,self.cable_len_list[i-1]])
                x[i*13 + 6] = 1
            self.cable_is_slack = self.isslack_multi(x[0:3].reshape((3,1))+ utilslib.QuatToRot(x[6:10]) @ self.pl_params.rho_vec_list, x[13*np.arange(1, self.nquad+1)+np.array([[0],[1],[2]])],self.pl_params.cable_length)
        
        # Third Scenario: Rigid link
        else: 
            print("Initalizing Rigidlink Scenario")
            self.cable_is_slack = np.zeros(self.nquad) # Dummy assignment (no cable for rigid link)
            # TODO how to deal with rho_vec_list (currently hardcode from rigid_links_mechanism.yaml
            self.cable_len_list = np.zeros((self.nquad, 1), dtype=float)

            # x = (26, 1) state init
            # s                 - 13 x 1, state vector = [xL, yL, zL, xLd, yLd, zLd, qw, qx, qy, qz, pL, qL, rL]
            # take_in/output    - 26 x 1, state vector(dot-elementwise) = [ xL,     yL,     zL,     
            #                                                               xLd,    yLd,    zLd,
                                                        # Name      Element Location
            x = np.array([0.0,  0.0,    0.0,            # pl pos    3
                          0.0,  0.0,    0.0,            # pl vel    6
                          1.0,  0.0,    0.0,    0.0,    # pl quat   10
                          0.0,  0.0,    0.0,            # pl omega  13
                          0.0,  0.0,    0.0,            # qd pos    16
                          0.0,  0.0,    0.0,            # qd vel    19 
                          1.0,  0.0,    0.0,    0.0,    # qd quat   23
                          0.0,  0.0,    0.0])           # qd omega  26
            qd_init = {}

            # init uav_F and uav_M
            qd_init["pos"] = x[0:3]
            qd_init["vel"] = x[3:6]
            qd_init["quat"] = x[19:23]
            qd_init["omega"] = x[10:13]
            qd_init["rot"] = utilslib.QuatToRot(qd_init["quat"]).T
                
            qd_init["pos_des"] = x[0:3]
            qd_init["vel_des"] = x[3:6]
            qd_init["acc_des"] = np.array([[0],[0],[0]])
            qd_init["jrk_des"] = np.array([[0],[0],[0]])
            qd_init["qd_yaw_des"] = 0
            qd_init["qd_yawdot_des"] = 0
            qd_init["quat_des"] = x[19:23]
            qd_init["omega_des"] = x[10:13]

            robot_trust_moment = self.rigid_links_cooperative_payload_controller_init(qd_init, self.pl_params)
            u = self.pl_params.A @ robot_trust_moment
            self.uav_F  = u[0] * qd_init["rot"][:,2].reshape((3,1))
            self.uav_M = u[1:4]
      
      # Second Scenario: Point Mass
      else:
        print("Initalizing Ptmass Scenario")
        ## init force to [sum of (quad and payload mass)] * gravity
        self.uav_F = np.array([(self.pl_params.mass + self.uav_params[0].mass) * self.uav_params[0].grav])

        ## init M to [0; 0; 0]
        self.uav_M = np.zeros((3,self.nquad), dtype=float)

        ## init calbe to taut
        self.cable_is_slack = np.zeros(self.nquad)

        ## init rho_vec_list = [] (empty)
        self.rho_vec_list = self.pl_params.rho_vec_list

        ## init cable_len_list = cable_length (read)
        self.cable_len_list = np.array(self.pl_params.cable_length)

        ## init state x as a (26, 1) state vector with inital position hard code to (0, 0, 0.5)
        # x = (26, 1) state init
                                                    # Name      Element Location
        x = np.array([0.0,  0.0,    0.0,            # pl pos    3
                      0.0,  0.0,    0.0,            # pl vel    6
                      1.0,  0.0,    0.0,    0.0,    # pl quat   10
                      0.0,  0.0,    0.0,            # pl omega  13
                      0.0,  0.0,    self.pl_params.cable_length,            # qd pos    16
                      0.0,  0.0,    0.0,            # qd vel    19 
                      1.0,  0.0,    0.0,    0.0,    # qd quat   23
                      0.0,  0.0,    0.0])           # qd omega  26

      # ROS Publisher 
      self.system_publisher = rospy.Publisher('system/marker',MarkerArray,queue_size=10)
      self.payload_odom_publisher = rospy.Publisher('payload/odom',Odometry,queue_size=1, tcp_nodelay=True)
      self.payload_odom_ground_truth_publisher = rospy.Publisher('payload/odom_ground_truth',Odometry,queue_size=1, tcp_nodelay=True)
      self.payload_path_publisher = rospy.Publisher('payload/path',Path,queue_size=1, tcp_nodelay=True)
      self.robot_vio_publisher = rospy.Publisher('payload/vio', Odometry, queue_size=1, tcp_nodelay=True)
      self.payload_path = Path()
      self.robot_odom_publisher = []
      self.attach_publisher = []
      self.robot_command_subscriber = []
      uav_type = []
      uav_in_team = []
      print(pl_params.uav_in_team)
      for i in range(self.nquad):
          uav_name = self.uav_params[i].uav_name
          uav_name_with_id = pl_params.uav_in_team[i]

          # ROS Publisher
          self.robot_odom_publisher.append(rospy.Publisher(uav_name_with_id + '/odom',Odometry, queue_size=1, tcp_nodelay=True))
          self.attach_publisher.append(rospy.Publisher(uav_name_with_id + '/attach',Odometry, queue_size=1, tcp_nodelay=True))
           
          # ROS Subscriber 
          controller_name = "/controller_" + str(i+1)
          self.robot_command_subscriber.append(rospy.Subscriber(controller_name + '/' + uav_name_with_id + '/rpm_cmd',RPMCommand,self.rpm_command_callback,i,queue_size=1, tcp_nodelay=True))
          self.robot_command_subscriber.append(rospy.Subscriber(controller_name + '/' + uav_name_with_id + '/fm_cmd',FMCommand,self.fm_command_callback,i,queue_size=1, tcp_nodelay=True))
    
      print("The list of uav is", sum(uav_in_team, []))
      # Visualization Init
      self.cable_marker_scale = 0.01 * np.ones(3)
      self.cable_marker_color = np.array([1.0,0.5,0.5,0.5])
      self.uav_marker_scale = 0.5 * np.ones(3)
      self.uav_marker_color = np.array([1.0,0.0,0.0,1.0])
      self.uav_mesh = []
      for idx in range(self.nquad):
        self.uav_mesh.append(self.uav_params[idx].mesh_path)

      if self.nquad == 1:
        self.payload_marker_scale = np.array([0.1,0.1,0.1])
      else:
        self.payload_marker_scale = np.ones(3)

      self.payload_marker_color = np.array([1.0,0.745,0.812,0.941])
      self.payload_mesh = self.pl_params.mesh_path
      if self.nquad == 1:
        self.payload_marker_msg = rosutilslib.init_marker_msg(Marker(),2,0,self.worldframe,self.payload_marker_scale,self.payload_marker_color,self.payload_mesh)
      else:
        self.payload_marker_msg = rosutilslib.init_marker_msg(Marker(),10,0,self.worldframe,self.payload_marker_scale,self.payload_marker_color,self.payload_mesh)
    
    ################################## Simulation Loop ################################
      try:
        self.hybrid_flag = rospy.get_param("hybrid_switch")
      except:
        rospy.set_param("hybrid_switch", True)
        self.hybrid_flag = rospy.get_param("hybrid_switch")
        print("The hybrid flag is ", self.hybrid_flag)

      if self.hybrid_flag:
        print("\n############################################################")
        print("HYBRID DYNAMICS IS ON")
        print("LOW PERFORMANCE PROCESSOR PROCEDE WITH CAUTION")
        print("############################################################\n")
        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            start = time.time()
            # Three scenario:
            #                 1. Cooperative
            #                 2. Point mass
            #                 3. Rigid link
            # dynamics solver is built separately for each scenario below
            # with if-else branch structure

            # Thrid Scenario: Rigid Link 
            if self.pl_params.id == "Rigid Link":
                sol = scipy.integrate.solve_ivp(self.rigid_links_cooperative_rigidbody_pl_EOM, t_span, x, method= 'RK45', t_eval=t_span)
                x = sol.y[:,-1]
            else:
            
            # Second Scenario: Point Mass
                if self.nquad == 1:
                    
                    ## first check for inelastic collision
                    pl_pos = x[0:3]
                    pl_vel = x[3:6]
                    robot_pos = x[13:16]
                    robot_vel = x[16:19]
                    cable_norm_vel = np.transpose(pl_pos - robot_pos) @ (pl_vel - robot_vel)/np.linalg.norm(pl_pos - robot_pos)
                    ## if collision, compute new velocities and assign to state
                    if cable_norm_vel > 1e-3 and not self.cable_is_slack:
                        print("Colliding")
                        v1, v2 = self.ptmass_inelastic_cable_collision(x[0:6], x[13:19], self.pl_params.mass, self.uav_params[0].mass)
                        x[3:6] = v1
                        x[16:19] = v2
                    
                    ## set up event for ivp solver
                    ptmasstautToSlack.terminal = True
                    ptmassslackToTaut.terminal = True
                    ptmasstautToSlack.direction = -1
                    ptmassslackToTaut.direction = 1
                    ptmassslackToTaut.cable_length = self.pl_params.cable_length
                    ptmasstautToSlack.cable_length = self.pl_params.cable_length

                    ## state integration
                    if self.cable_is_slack:
                        print("Cable is slack")
                        #print(x)
                        sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmassslackToTaut)
                    else:
                        #print("Cable is taut")
                        #print(x)
                        sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmasstautToSlack)
                    
                    ## extract state from solver soltion
                    if (np.all(x==sol.y[:, -1])) and (len(sol.y_events[0]) != 0):
                        x = sol.y_events[0][:]
                        x = x.T
                        x = x.reshape((x.shape[0],))
                    else:
                        x = sol.y[:,-1]
                    
                    ## recheck cable slack condition
                    self.cable_is_slack = self.isslack(x[0:3], x[13:16], self.pl_params.cable_length)
                
            # First Scenario: Cooperative 
                else:    
                    ## first, we check for collision
                    inelastic_collision_flag, xi = self.cooperative_check_inelastic(x)
                    
                    ## make sure velocities are distributed with no new collisions happening 
                    while np.any(inelastic_collision_flag):
                        print("collision!")
                        before_collide_inelastic_collision_flag = inelastic_collision_flag
                        print("The before inelastic collision_flag is", before_collide_inelastic_collision_flag)
                        x = self.rigidbody_quad_inelastic_cable_collision(x, inelastic_collision_flag)
                        print("collision finished!")
                        after_collide_inelastic_collision_flag, xi = self.cooperative_check_inelastic(x)
                        print("Checking after collision")
                        print("The after inelastic collision_flag is", after_collide_inelastic_collision_flag)
                        if np.any((after_collide_inelastic_collision_flag - before_collide_inelastic_collision_flag)>0):
                            inelastic_collision_flag = after_collide_inelastic_collision_flag + before_collide_inelastic_collision_flag
                            for i in range(inelastic_collision_flag.shape[0]):
                                if inelastic_collision_flag[i] != 0:
                                    inelastic_collision_flag[i] = 1.0
                        else:
                            print("Seems to be last time of the collision")
                            attach_point_position = x[0:3].reshape((3,1))+ utilslib.QuatToRot(x[6:10]) @ self.pl_params.rho_vec_list
                            inelastic_collision_flag = after_collide_inelastic_collision_flag
                            for i in range(self.nquad):
                                print("The robot ", i+1, " position is ", x[(i+1)*13:(i+1)*13+3])
                                print("The xi ", i+1, " is ", xi[:,i])
                                print("The new robot position is, ", attach_point_position[:,i] - self.pl_params.cable_length[i] * xi[:,i])
                                x[(i+1)*13:(i+1)*13+3] = attach_point_position[:,i] - self.pl_params.cable_length[i] * xi[:,i]
                    
                    #print("                                              ")
                    ## set up event for ivp solver
                    slack_condition = self.cable_is_slack
                    idx = np.arange(1, self.pl_params.nquad+1)
                    slack_cable_idx = idx[slack_condition == 1]
                    taut_cable_idx = idx[slack_condition == 0]
                    num_of_slack_cable = np.max(slack_cable_idx.shape)
                    num_of_taut_cable = np.max(taut_cable_idx.shape)

                    GuardEvents = []
                    for i in range(self.nquad):
                        GuardEvents.append(lambda t, x: cooperativeGuard(t, x, GuardEvents[i].pl_params.nquad, GuardEvents[i].slack_condition, GuardEvents[i].pl_params.rho_vec_list, GuardEvents[i].pl_params.cable_length, GuardEvents[i].i))

                    temp = np.zeros((slack_condition.shape), dtype=float)
                    for i in range(self.nquad):
                        if slack_condition[i]:
                            temp[i] = 1.0
                        else:
                            temp[i] = -1.0
                    id = 0
                    for fcn in GuardEvents:
                        fcn.terminal = True
                        if num_of_slack_cable == 0:
                            fcn.direction = -1.0
                        elif num_of_taut_cable == 0:
                            fcn.direction = 1.0
                        else:
                            fcn.direction = temp[id]
                        fcn.pl_params = self.pl_params
                        fcn.slack_condition = slack_condition
                        fcn.i = id
                        id = id + 1

                    ## state integration
                    sol = scipy.integrate.solve_ivp(self.hybrid_cooperative_rigidbody_pl_transportationEOM, t_span, x, method='RK23', t_eval=t_span, events=GuardEvents)
                    
                    ## extract state from solver soltion
                    EventTriggered_bool = sol.status
                    EventTriggered_id = 0

                    if EventTriggered_bool == 1:
                        for i in range(self.nquad):
                            if len(sol.y_events[i]) != 0: 
                                EventTriggered_id = i
                        if (np.all(x==sol.y[:, -1])):
                            x = sol.y_events[EventTriggered_id][:]
                            x = x.T
                            x = x.reshape((x.shape[0],))
                    else:
                        x = sol.y[:,-1]
                        # print("Getting x here: ", x)
                    
                    self.cable_is_slack = self.isslack_multi(x[0:3].reshape((3,1))+ utilslib.QuatToRot(x[6:10]) @ self.pl_params.rho_vec_list, x[13*np.arange(1, self.nquad+1)+np.array([[0],[1],[2]])],self.pl_params.cable_length)
                    
            end = time.time()
            
            
            # Publish payload odometry

            payload_odom = Odometry()
            # current_time = rospy.get_rostime()
            payload_odom.header.stamp = current_time
            payload_odom.header.frame_id = self.worldframe
            
            payload_vio = Odometry()        
            # current_time = rospy.get_rostime()  
            payload_vio.header.stamp = current_time
            payload_vio.header.frame_id = self.worldframe 
            payload_rotmat = utilslib.QuatToRot(sol.y[:,0][6:10])

            if self.pl_params.mechanism_type == 'Rigid Link':
                load_pos = x[0:3].reshape((3,1)) + payload_rotmat @ self.pl_params.rho_load
            else:
                load_pos = x[0:3].reshape((3,1))

            white_noise = np.random.multivariate_normal(np.zeros(12),white_noise_cov)
            deltaR = utilslib.expSO3(white_noise[6:9])
            payload_rot_with_noise = np.matmul(rot_math.from_quat([x[7],x[8],x[9],x[6]]).as_matrix(), deltaR)
            payload_quat_with_noise = rot_math.from_matrix(payload_rot_with_noise).as_quat()

            payload_odom.pose.pose.position.x = load_pos[0] + white_noise[0]
            payload_odom.pose.pose.position.y = load_pos[1] + white_noise[1]
            payload_odom.pose.pose.position.z = load_pos[2] + white_noise[2]
            payload_odom.twist.twist.linear.x    = x[3] + white_noise[3]
            payload_odom.twist.twist.linear.y    = x[4] + white_noise[4]
            payload_odom.twist.twist.linear.z    = x[5] + white_noise[5]
            payload_odom.pose.pose.orientation.w = payload_quat_with_noise[3] 
            payload_odom.pose.pose.orientation.x = payload_quat_with_noise[0]
            payload_odom.pose.pose.orientation.y = payload_quat_with_noise[1]
            payload_odom.pose.pose.orientation.z = payload_quat_with_noise[2]
            payload_odom.twist.twist.angular.x   = x[10] + white_noise[9]
            payload_odom.twist.twist.angular.y   = x[11] + white_noise[10]
            payload_odom.twist.twist.angular.z   = x[12] + white_noise[11]

            self.payload_odom_publisher.publish(payload_odom)

            payload_odom.pose.pose.position.x = load_pos[0]
            payload_odom.pose.pose.position.y = load_pos[1]
            payload_odom.pose.pose.position.z = load_pos[2]
            payload_odom.twist.twist.linear.x    = x[3]
            payload_odom.twist.twist.linear.y    = x[4]
            payload_odom.twist.twist.linear.z    = x[5]
            payload_odom.pose.pose.orientation.w = x[6] 
            payload_odom.pose.pose.orientation.x = x[7]
            payload_odom.pose.pose.orientation.y = x[8]
            payload_odom.pose.pose.orientation.z = x[9]
            payload_odom.twist.twist.angular.x   = x[10]
            payload_odom.twist.twist.angular.y   = x[11]
            payload_odom.twist.twist.angular.z   = x[12]

            self.payload_odom_ground_truth_publisher.publish(payload_odom)

            # Publish payload path
            # current_time = rospy.get_rostime()

            self.payload_path.header.stamp = current_time
            self.payload_path.header.frame_id = self.worldframe 

            pl_pose_stamped = PoseStamped()
            # current_time = rospy.get_rostime()
            pl_pose_stamped.header.stamp = current_time
            pl_pose_stamped.header.frame_id = self.worldframe
            pl_pose_stamped.pose = payload_odom.pose.pose 

            self.payload_path.poses.append(pl_pose_stamped)
            self.payload_path_publisher.publish(self.payload_path)

            system_marker = MarkerArray()
            cable_point_list = np.zeros((2*self.nquad,3))

            #uav_odom = Odometry()
            #attach_odom = Odometry()
            #ukf_measure = ukf_measurement_update()
            #self.UKF_publisher[self.nquad+1].publish(ukf_measure)
            #self.robot_odom_publisher[self.nquad+1].publish(uav_odom)
            #self.attach_publisher[self.nquad+1].publish(attach_odom)
            #self.UKF_publisher[self.nquad].publish(ukf_measure)
            #self.robot_odom_publisher[self.nquad].publish(uav_odom)
            #self.attach_publisher[self.nquad].publish(attach_odom)

            for uav_id in range(self.nquad-1, -1, -1):
                if self.pl_params.mechanism_type == 'Rigid Link':
                    print("I'm in Rigid Link", uav_id)
                    uav_state = x[13:26]
                    attach_pos = x[0:3] + payload_rotmat @ (self.pl_params.rho_robot[:,uav_id]+np.array([0.0,0,0.04]))
                    uav_state[0:3] = attach_pos
                    attach_vel = uav_state[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.pl_params.rho_robot[:,uav_id]))
                    if not self.cable_is_slack[uav_id]:
                        uav_attach_vector = uav_state[0:3] - attach_pos[0:3]
                        uav_attach_distance = np.linalg.norm(uav_attach_vector)
                        if uav_attach_distance > self.cable_len_list[uav_id]:
                            xi = uav_attach_vector/uav_attach_distance
                            uav_state[0:3] = attach_pos[0:3] + self.cable_len_list[uav_id] * xi
                else:
                    # print("I'm in non rigid link", uav_id)
                    uav_state = x[self.pl_dim_num+self.uav_dim_num*uav_id:self.pl_dim_num+self.uav_dim_num*(uav_id+1)]
                    attach_pos = x[0:3] + np.matmul(payload_rotmat, self.rho_vec_list[:,uav_id])
                    attach_vel = x[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.rho_vec_list[:,uav_id]))
                    if not self.cable_is_slack[uav_id]:
                        uav_attach_vector = uav_state[0:3] - attach_pos[0:3]
                        uav_attach_distance = np.linalg.norm(uav_attach_vector)
                        if uav_attach_distance > self.cable_len_list[uav_id]:
                            xi = uav_attach_vector/uav_attach_distance
                            uav_state[0:3] = attach_pos[0:3] + self.cable_len_list[uav_id] * xi
                        
                white_noise = np.random.multivariate_normal(np.zeros(12),uav_white_noise_cov)
                deltaR = utilslib.expSO3(white_noise[6:9])
                uav_rot_with_noise = np.matmul(rot_math.from_quat([uav_state[7],uav_state[8],uav_state[9],uav_state[6]]).as_matrix(), deltaR)
                uav_quat_with_noise = rot_math.from_matrix(uav_rot_with_noise).as_quat()

                # Publish UAV odometry
                uav_odom = Odometry()
                uav_odom.header.stamp = current_time
                uav_odom.header.frame_id = self.worldframe 
                uav_odom.pose.pose.position.x = uav_state[0] + white_noise[0]
                uav_odom.pose.pose.position.y = uav_state[1] + white_noise[1]
                uav_odom.pose.pose.position.z = uav_state[2] + white_noise[2]
                uav_odom.twist.twist.linear.x    = uav_state[3] + white_noise[3]
                uav_odom.twist.twist.linear.y    = uav_state[4] + white_noise[4]
                uav_odom.twist.twist.linear.z    = uav_state[5] + white_noise[5]
                uav_odom.pose.pose.orientation.w = uav_quat_with_noise[3]
                uav_odom.pose.pose.orientation.x = uav_quat_with_noise[0]
                uav_odom.pose.pose.orientation.y = uav_quat_with_noise[1]
                uav_odom.pose.pose.orientation.z = uav_quat_with_noise[2]
                uav_odom.twist.twist.angular.x   = uav_state[10] + white_noise[9]     
                uav_odom.twist.twist.angular.y   = uav_state[11] + white_noise[10]    
                uav_odom.twist.twist.angular.z   = uav_state[12] + white_noise[11]    
                self.robot_odom_publisher[uav_id].publish(uav_odom)

                # Publish UAV attach odometry
                attach_odom = Odometry()
                attach_odom.header.stamp = current_time
                attach_odom.header.frame_id = self.worldframe 
                attach_odom.pose.pose.position.x = attach_pos[0]
                attach_odom.pose.pose.position.y = attach_pos[1]
                attach_odom.pose.pose.position.z = attach_pos[2]
                attach_odom.twist.twist.linear.x = attach_vel[0]
                attach_odom.twist.twist.linear.y = attach_vel[1]
                attach_odom.twist.twist.linear.z = attach_vel[2]
                self.attach_publisher[uav_id].publish(attach_odom)

                cable_point_list[2*uav_id,:] = uav_state[0:3]
                cable_point_list[2*uav_id+1,:] = attach_pos[0:3]
                uav_marker_msg = rosutilslib.init_marker_msg(Marker(),10,0,self.worldframe,self.uav_marker_scale,self.uav_marker_color,self.uav_mesh[uav_id])
                uav_marker = rosutilslib.update_marker_msg(uav_marker_msg,uav_state[0:3],uav_state[6:10],uav_id)
                system_marker.markers.append(uav_marker)

            # Update cable visualization
            cable_marker_msg = rosutilslib.init_marker_msg(Marker(),5,0,self.worldframe,self.cable_marker_scale,self.cable_marker_color)
            system_marker.markers.append(rosutilslib.update_line_msg(cable_marker_msg,cable_point_list,self.nquad + 1))
            # Update payload visualization
            system_marker.markers.append(rosutilslib.update_marker_msg(self.payload_marker_msg,x[0:3],x[6:10],self.nquad+2))
            self.system_publisher.publish(system_marker)

            rate.sleep()    
      elif self.hybrid_flag == False:
        print("\n############################################################")
        print("HYBRID DYNAMICS IS OFF")
        print("############################################################\n")
        while not rospy.is_shutdown():
            start = time.time()
            if self.pl_params.id == "Rigid Link":
                sol = scipy.integrate.solve_ivp(self.rigid_links_cooperative_rigidbody_pl_EOM, t_span, x, method= 'RK45', t_eval=t_span)
            else:
                if self.nquad == 1:
                    sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span)
                else:    
                    sol = scipy.integrate.solve_ivp(self.hybrid_cooperative_rigidbody_pl_transportationEOM, t_span, x, method='RK45', t_eval=t_span)
            end = time.time()
            x = sol.y[:,1]

            # Publish payload odometry
            current_time = rospy.get_rostime()
            payload_odom = Odometry()
            payload_vio = Odometry()   
            payload_odom.header.stamp = current_time
            payload_odom.header.frame_id = self.worldframe 
            payload_rotmat = utilslib.QuatToRot(sol.y[:,0][6:10])

            if self.pl_params.mechanism_type == 'Rigid Link':
                load_pos = x[0:3].reshape((3,1)) + payload_rotmat @ self.pl_params.rho_load
            else:
                load_pos = x[0:3].reshape((3,1)) 

            white_noise = np.random.multivariate_normal(np.zeros(12),white_noise_cov)
            deltaR = utilslib.expSO3(white_noise[6:9])
            payload_rot_with_noise = np.matmul(rot_math.from_quat([x[7],x[8],x[9],x[6]]).as_matrix(), deltaR)
            payload_quat_with_noise = rot_math.from_matrix(payload_rot_with_noise).as_quat()

            payload_odom.pose.pose.position.x = load_pos[0] + white_noise[0]
            payload_odom.pose.pose.position.y = load_pos[1] + white_noise[1]
            payload_odom.pose.pose.position.z = load_pos[2] + white_noise[2]
            payload_odom.twist.twist.linear.x    = x[3] + white_noise[3]
            payload_odom.twist.twist.linear.y    = x[4] + white_noise[4]
            payload_odom.twist.twist.linear.z    = x[5] + white_noise[5]
            payload_odom.pose.pose.orientation.w = payload_quat_with_noise[3] 
            payload_odom.pose.pose.orientation.x = payload_quat_with_noise[0]
            payload_odom.pose.pose.orientation.y = payload_quat_with_noise[1]
            payload_odom.pose.pose.orientation.z = payload_quat_with_noise[2]
            payload_odom.twist.twist.angular.x   = x[10] + white_noise[9]
            payload_odom.twist.twist.angular.y   = x[11] + white_noise[10]
            payload_odom.twist.twist.angular.z   = x[12] + white_noise[11]

            self.payload_odom_publisher.publish(payload_odom)

            payload_odom.pose.pose.position.x = load_pos[0]
            payload_odom.pose.pose.position.y = load_pos[1]
            payload_odom.pose.pose.position.z = load_pos[2]
            payload_odom.twist.twist.linear.x    = x[3]
            payload_odom.twist.twist.linear.y    = x[4]
            payload_odom.twist.twist.linear.z    = x[5]
            payload_odom.pose.pose.orientation.w = x[6] 
            payload_odom.pose.pose.orientation.x = x[7]
            payload_odom.pose.pose.orientation.y = x[8]
            payload_odom.pose.pose.orientation.z = x[9]
            payload_odom.twist.twist.angular.x   = x[10]
            payload_odom.twist.twist.angular.y   = x[11]
            payload_odom.twist.twist.angular.z   = x[12]

            self.payload_odom_ground_truth_publisher.publish(payload_odom)

            self.payload_odom_publisher.publish(payload_odom)
            self.robot_vio_publisher.publish(payload_vio)
            self.payload_ctrl_wrench.publish(self.wrench_value)
            # Publish payload path
            current_time = rospy.get_rostime()

            self.payload_path.header.stamp = current_time
            self.payload_path.header.frame_id = self.worldframe 
            payload_rotmat = utilslib.QuatToRot(sol.y[:,0][6:10])

            pl_pose_stamped = PoseStamped()
            current_time = rospy.get_rostime()
            pl_pose_stamped.header.stamp = current_time
            pl_pose_stamped.header.frame_id = self.worldframe
            pl_pose_stamped.pose = payload_odom.pose.pose 

            self.payload_path.poses.append(pl_pose_stamped)
            self.payload_path_publisher.publish(self.payload_path)

            system_marker = MarkerArray()
            cable_point_list = np.zeros((2*self.nquad,3))
            
            for uav_id in range(self.nquad-1, -1, -1):
                if self.pl_params.mechanism_type == 'Rigid Link':
                    uav_state = x[13:26]
                    attach_pos = x[0:3] + payload_rotmat @ (self.pl_params.rho_robot[:,uav_id]+np.array([-0.06,0,0.02]))
                    uav_state[0:3] = attach_pos
                    attach_vel = uav_state[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.pl_params.rho_robot[:,uav_id]))
                    if not self.cable_is_slack[uav_id]:
                        uav_attach_vector = uav_state[0:3] - attach_pos[0:3]
                        uav_attach_distance = np.linalg.norm(uav_attach_vector)
                        if uav_attach_distance > self.cable_len_list[uav_id]:
                            xi = uav_attach_vector/uav_attach_distance
                            uav_state[0:3] = attach_pos[0:3] + self.cable_len_list[uav_id] * xi
                        
                else:
                    uav_state = x[self.pl_dim_num+self.uav_dim_num*uav_id:self.pl_dim_num+self.uav_dim_num*(uav_id+1)]
                    attach_pos = x[0:3] + np.matmul(payload_rotmat, self.rho_vec_list[:,uav_id])
                    attach_vel = x[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.rho_vec_list[:,uav_id]))
                    if not self.cable_is_slack[uav_id]:
                        uav_attach_vector = uav_state[0:3] - attach_pos[0:3]
                        uav_attach_distance = np.linalg.norm(uav_attach_vector)
                        if uav_attach_distance > self.cable_len_list[uav_id]:
                            xi = uav_attach_vector/uav_attach_distance
                            uav_state[0:3] = attach_pos[0:3] + self.cable_len_list[uav_id] * xi
                        
                white_noise = np.random.multivariate_normal(np.zeros(12),uav_white_noise_cov)
                deltaR = utilslib.expSO3(white_noise[6:9])
                uav_rot_with_noise = np.matmul(rot_math.from_quat([uav_state[7],uav_state[8],uav_state[9],uav_state[6]]).as_matrix(), deltaR)
                uav_quat_with_noise = rot_math.from_matrix(uav_rot_with_noise).as_quat()

                # Publish UAV odometry
                uav_odom = Odometry()
                uav_odom.header.stamp = current_time
                uav_odom.header.frame_id = self.worldframe 
                uav_odom.pose.pose.position.x = uav_state[0] + white_noise[0]
                uav_odom.pose.pose.position.y = uav_state[1] + white_noise[1]
                uav_odom.pose.pose.position.z = uav_state[2] + white_noise[2]
                uav_odom.twist.twist.linear.x    = uav_state[3] + white_noise[3]
                uav_odom.twist.twist.linear.y    = uav_state[4] + white_noise[4]
                uav_odom.twist.twist.linear.z    = uav_state[5] + white_noise[5]
                uav_odom.pose.pose.orientation.w = uav_quat_with_noise[3]
                uav_odom.pose.pose.orientation.x = uav_quat_with_noise[0]
                uav_odom.pose.pose.orientation.y = uav_quat_with_noise[1]
                uav_odom.pose.pose.orientation.z = uav_quat_with_noise[2]
                uav_odom.twist.twist.angular.x   = uav_state[10] + white_noise[9]     
                uav_odom.twist.twist.angular.y   = uav_state[11] + white_noise[10]    
                uav_odom.twist.twist.angular.z   = uav_state[12] + white_noise[11]    
                self.robot_odom_publisher[uav_id].publish(uav_odom)

                # Publish UAV attach odometry
                attach_odom = Odometry()
                attach_odom.header.stamp = current_time
                attach_odom.header.frame_id = self.worldframe 
                attach_odom.pose.pose.position.x = attach_pos[0]
                attach_odom.pose.pose.position.y = attach_pos[1]
                attach_odom.pose.pose.position.z = attach_pos[2]
                attach_odom.twist.twist.linear.x = attach_vel[0]
                attach_odom.twist.twist.linear.y = attach_vel[1]
                attach_odom.twist.twist.linear.z = attach_vel[2]
                self.attach_publisher[uav_id].publish(attach_odom)

                cable_point_list[2*uav_id,:] = uav_state[0:3]
                cable_point_list[2*uav_id+1,:] = attach_pos[0:3]
                uav_marker_msg = rosutilslib.init_marker_msg(Marker(),10,0,self.worldframe,self.uav_marker_scale,self.uav_marker_color,self.uav_mesh[uav_id])
                uav_marker = rosutilslib.update_marker_msg(uav_marker_msg,uav_state[0:3],uav_state[6:10],uav_id)
                system_marker.markers.append(uav_marker)

            # Update cable visualization
            cable_marker_msg = rosutilslib.init_marker_msg(Marker(),5,0,self.worldframe,self.cable_marker_scale,self.cable_marker_color)
            system_marker.markers.append(rosutilslib.update_line_msg(cable_marker_msg,cable_point_list,uav_id + 1))

            # Update payload visualization
            system_marker.markers.append(rosutilslib.update_marker_msg(self.payload_marker_msg,load_pos[0:3],x[6:10],uav_id+2))
            self.system_publisher.publish(system_marker)

            rate.sleep()

  def istaut(self, robot_pos, attach_pos, cable_length):
    # DESCRIPTION:
    # This method takes in quad rotor position, cable attchment position
    # and cable and determines if the cable is taut or not.
    # This method is only used in cooperative suspended payload case
    
    # INPUTS:
    # robot_pos                 - a [3 by (# of robots)] ndarray, the n-th col describes the 3D position
    #                             of the n-th MAV position 
    # attach_pos                - a [3 by (# of robots)] ndarray, the n-th col describes the 3D position
    #                             of the n-th cable attachment position
    # cable_length              - a (# of robots) sized ndarray (shape of (3,)), the n-th element describes the length
    #                             of the cable used by the n-th robot

    # OUTPUTS:
    # flag                      - a (# of robots) sized ndarray (shape of (3,)) with boolean type elements
    #                             the n-th element describes if the n-th cable is taut
      flag = (np.linalg.norm(robot_pos - attach_pos, 2, 0) > (cable_length - 1e-3))
      #print("The distance is", np.linalg.norm(robot_pos-attach_pos, 2, 0))
      #print("Taut condition is ", flag)
      return flag

  def isslack_multi(self, robot_pos, attach_pos, cable_length):
    # DESCRIPTION:
    # This method takes in quad rotor position, cable attchment position
    # and cable and determines if the cable is slack or not.
    # This method is only used in cooperative suspended payload case
    
    # INPUTS:
    # robot_pos                 - a [3 by (# of robots)] ndarray, the n-th col describes the 3D position
    #                             of the n-th MAV position 
    # attach_pos                - a [3 by (# of robots)] ndarray, the n-th col describes the 3D position
    #                             of the n-th cable attachment position
    # cable_length              - a (# of robots) sized ndarray (shape of (3,)), the n-th element describes the length
    #                             of the cable used by the n-th robot

    # OUTPUTS:
    # flag                      - a (# of robots) sized ndarray (shape of (3,)) with boolean type elements
    #                             the n-th element describes if the n-th cable is slack
      flag = (np.linalg.norm(robot_pos - attach_pos, 2, 0) <= (cable_length - 1e-3))
      return flag

  def isslack(self, robot_pos, attach_pos, cable_length):
    # DESCRIPTION:
    # This method takes in quad rotor position, cable attchment position
    # and cable and determines if the cable is slack or not.
    # This method is only used in for point mass case
    
    # INPUTS:
    # robot_pos                 - a [3 by 1] ndarray, describing the 3D position
    #                             of the MAV position 
    # attach_pos                - a [3 by 1] ndarray, describing the 3D position
    #                             of the cable attachment position
    # cable_length              - a ndarray (shape of (1,)), the n-th element describes the length
    #                             of the cable used by the n-th robot

    # OUTPUTS:
    # flag                      - an ndarray (shape of (1,)) with boolean type elements
    #                             the element describes if the cable is slack
      if (np.linalg.norm(robot_pos - attach_pos) > (cable_length - 1e-3)):
        return np.array([0.0])
      else:
        return np.array([1.0])

####################################################################################
##################                 cooperative                  ####################
####################################################################################
  def hybrid_cooperative_rigidbody_pl_transportationEOM(self, t, s): 
      # DESCRIPTION:
      # EOM Wrapper function for solving quadrotor equation of motion
      #	EOM takes in time, state vector
      #	and parameters and output the derivative of the state vector, the
      #	actual calcution is done in quadEOM_readonly.
          
      # INPUTS:
      # t             - 1 x 1, time
      # s             - (7*nquad + 6*nquad + 13) x 1,
      #                 state vector = [xL, yL, zL, xLd, yLd, zLd,
      #                                 qLw, qLx, qLy, qLz, pL, qL, rL,
      #                                 [xQ, yQ, zQ, xQd, yQd, zQd]_i, i = 1,...,nquad
      #                                 [qw, qx, qy, qz, pQuad, qQuad, rQuad]_i, i = 1,...,nquad

      # IMPORTANT ATTRIBUTES USED
      # nquad         - number of quads
      # qn            - quad id (used for multi-robot simulations)
      # plcontrolhdl  - function handle of payload controller
      # qdcontrolhdl  - function handle of quad attitude controller
      # trajhandle    - function handle of payload trajectory generator
      # params        - struct, output from crazyflie() and whatever parameters you want to pass in
      
      # OUTPUTS:
      # sdot          - 13*(nquad+1) x 1, derivative of state vector s
      if not self.sim_start :
          self.pl_accel = np.zeros(3)
          self.pl_ang_accel = np.zeros(3)
          self.attach_accel = np.zeros(3)
          self.sim_start = True
      
      # convert payload state
      pl_state = s[0:self.pl_dim_num]
      pl_pos = pl_state[0:3]
      pl_vel = pl_state[3:6]
      pl_quat = pl_state[6:10]
      pl_rot = utilslib.QuatToRot(pl_quat) # This needs to be the rotation matrix of the payload w.r.t the world frame
      pl_omg = pl_state[10:13]
      pl_omg_asym = utilslib.vec2asym(pl_omg)

      # convert UAV state
      qd_state = np.reshape(s[self.pl_dim_num:],(self.nquad,13))
      qd_pos = qd_state[:,0:3]
      qd_vel = qd_state[:,3:6]
      qd_quat = qd_state[:,6:10]
      
      robot_attach_vector = pl_pos + np.matmul(pl_rot, self.rho_vec_list).T - qd_pos
      qd_xi = robot_attach_vector/linalg.norm(robot_attach_vector,axis=1)[:, np.newaxis] 
      qd_xidot = (pl_vel + np.matmul(pl_rot, np.matmul(pl_omg_asym, self.rho_vec_list)).T - qd_vel) / self.cable_len_list[:, np.newaxis]
      cable_omg = np.cross(qd_xi,qd_xidot)
      attach_centrifugal_accel = np.matmul(pl_omg_asym, np.matmul(pl_omg_asym, self.rho_vec_list))
      self.attach_accel = self.pl_accel + np.array([0,0,self.pl_params.grav]) + \
                          np.matmul(pl_rot, np.matmul(utilslib.vec2asym(self.pl_ang_accel), self.rho_vec_list)).T + \
                          np.matmul(pl_rot, attach_centrifugal_accel).T

      ## payload dynamics
      ML = self.pl_params.mass * np.eye(3)
      C = np.zeros((3,3))
      D = np.zeros((3,3))
      E = np.zeros((3,3))
      pl_net_F = np.zeros(3)
      pl_net_M = np.zeros(3)
      uav_u = np.zeros((3,self.nquad))
      tension_vector = np.zeros((3,self.nquad))

      for uav_idx in range(self.nquad):
          xi = qd_xi[uav_idx,:]
          xixiT = np.matmul(xi[:,np.newaxis], xi[:,np.newaxis].T)
          xidot = qd_xidot[uav_idx,:]
          rho_qn_asym = self.pl_params.P[3:6,3*uav_idx:3*uav_idx+3]
          uav_rot = utilslib.QuatToRot(qd_quat[uav_idx,:])
          cbl_omg = cable_omg[uav_idx,:]
          qd_u = np.matmul(uav_rot, np.array([0,0,self.uav_F[uav_idx]]))
          uav_u[:,uav_idx] = qd_u
          u_parallel = np.matmul(xixiT, qd_u)

          if not self.cable_is_slack[uav_idx]:
              cable_len = self.cable_len_list[uav_idx]
              # If the cable is taut, calculate these terms

              # Force and Moment at attach point
              attach_qn_force = u_parallel - self.uav_params[uav_idx].mass * cable_len * linalg.norm(cbl_omg)**2 * xi - \
                                self.uav_params[uav_idx].mass * np.matmul(xixiT, np.matmul(pl_rot, attach_centrifugal_accel[:,uav_idx]))
              attach_qn_moment = np.matmul(rho_qn_asym, np.matmul(pl_rot.T, attach_qn_force))

              # Tension Force on each cable
              tension = self.uav_params[uav_idx].mass*cable_len*np.sum(cbl_omg**2) - \
                        np.matmul(xi, qd_u - self.uav_params[uav_idx].mass * self.attach_accel[uav_idx,:])
              tension_vector[:,uav_idx] = tension * xi

              # Sum Net Force, Moment and other corresponding terms for the Payload Dynamics
              pl_net_F = pl_net_F + attach_qn_force
              pl_net_M = pl_net_M + attach_qn_moment
              Ck = self.uav_params[uav_idx].mass * np.matmul(rho_qn_asym, np.matmul(pl_rot.T, xixiT))
              Dk = - np.transpose(Ck)
              Ek = np.matmul(Ck, np.matmul(pl_rot, rho_qn_asym))
              C = C + Ck
              D = D + Dk
              E = E + Ek
              ML = ML + self.uav_params[uav_idx].mass * xixiT
      
      invML = linalg.inv(ML)


      payload_rot = utilslib.QuatToRot(s[6:10])
      diag_rot = np.zeros((0,0), dtype=float)
      for i in range(1, self.nquad+1):
          diag_rot = LA.block_diag(diag_rot, payload_rot)
      
      #pl_ctrl_F = pl_net_F
      #pl_ctrl_M = pl_net_M
      pl_net_F = pl_net_F # pl_net_F is in world frame
      pl_net_M = pl_net_M # pl_net_M is in world frame
      mu_ext =  diag_rot @  self.pl_params.pseudo_inv_P @ np.append(self.ext_force, self.ext_torque, axis=0)
      # print("mu_ext is\n", mu_ext)
      ## Dynamics of Payload


      # print(pl_net_F)
      sdotLoad = self.rigidbody_payloadEOM_readonly(pl_state,pl_net_F,pl_net_M,invML,C,D,E)
      
      self.pl_accel = sdotLoad[3:6]
      self.pl_ang_accel = sdotLoad[10:13]
      
      sdot = sdotLoad
      
      for uav_idx in range(self.nquad):
         uav_s = qd_state[uav_idx,:] 
         if self.cable_is_slack[uav_idx]:
             F = self.uav_F[uav_idx]
             M = self.uav_M[:,uav_idx]
             sdotQuad = self.slack_quadEOM_readonly(uav_s,F,M,uav_idx)
         else:
             #print(tension_vector[:,uav_idx])
             #print(xixiT)
             #T = tension_vector[:,uav_idx] - np.matmul(xixiT, mu_ext[uav_idx*self.nquad:(uav_idx+1)*self.nquad])
             T = tension_vector[:,uav_idx]
             
             #test = mu_ext[uav_idx*self.nquad:(uav_idx+1)*self.nquad]
             #print(test.shape)
             #print(test)
             #test2 = self.uav_F[uav_idx]
             #print(test2.shape)
             #print(test2)
             F = self.uav_F[uav_idx] 
             M = self.uav_M[:,uav_idx]
             sdotQuad = self.taut_quadEOM_readonly(uav_s,F,M,T,uav_idx)
         sdot = np.concatenate((sdot,sdotQuad))
        

      return sdot
    
  def rigidbody_payloadEOM_readonly(self, s, F, M, invML, C, D, E): 
      # DESCRIPTION:
      # rigidbody_payloadEOM_READONLY Solve payload equation of motion
      # payloadEOM_readonly calculate the derivative of the state vector
      
      # INPUTS:
      # t      - 1 x 1, time
      # s      - 13 x 1, state vector = [xL, yL, zL, xLd, yLd, zLd, qw, qx, qy, qz, pL,qL,rL]
      # F      - 1 x 1, thrust output from controller (only used in simulation)
      # M      - 3 x 1, moments output from controller (only used in simulation)
      # invML  - 3 x 3, inverse of the ML with payload mass as diagnal elements
      # C      - coeff
      # D      - coeff
      # E      - coeff

      # IMPORTANT ATTRIBUTE USED
      # params - struct, output from crazyflie() and whatever parameters you want to pass in
          
      # OUTPUTS:
      # sdot   - 13 x 1, derivative of state vector s
      
      #************ EQUATIONS OF MOTION ************************
      
      # Assign states
      quat = s[6:10]
      omega = s[10:13]
      qLdot = utilslib.quat_dot(quat,omega) 

      # Payload Angular acceleration
      effective_M = M - np.matmul(C, np.matmul(invML, F)) - np.cross(omega, np.matmul(self.pl_params.I, omega))
      effective_inertia = self.pl_params.I + np.matmul(C, np.matmul(invML, D)) - E
      omgLdot = np.linalg.solve(effective_inertia,effective_M) + self.pl_params.invI @ self.ext_torque 
      #omgLdot = np.linalg.solve(effective_inertia,effective_M)

      # Payload Acceleration
      accL = np.matmul(invML, F + np.matmul(D, omgLdot)) - np.array([0,0,self.pl_params.grav]) + self.ext_force/self.pl_params.mass
      # accL = np.matmul(invML, F + np.matmul(D, omgLdot)) - np.array([0,0,self.pl_params.grav])
      temp = invML @ (F * self.pl_params.mass)
      self.wrench_value.force.x = temp[0]
      self.wrench_value.force.y = temp[1]
      self.wrench_value.force.z = temp[2]
      self.wrench_value.torque.x = M[0]
      self.wrench_value.torque.y = M[1]
      self.wrench_value.torque.z = M[2]
      # Assemble sdot
      sdotLoad = np.zeros(self.pl_dim_num)
      sdotLoad[0:3] = s[3:6] 
      sdotLoad[3:6] = accL
      sdotLoad[6:10] = qLdot
      sdotLoad[10:13] = omgLdot

      return sdotLoad

  def taut_quadEOM_readonly(self, qd, F, M, T, uav_idx): 
      # DESCRIPTION:
      # QUADEOM_READONLY Solve quadrotor equation of motion when the cable is
      # taut.
      # quadEOM_readonly calculate the derivative of the state vector
          
      # INPUTS:
      # qd - struct, quadrotor parameters you want to pass in
      # F - ndarray, thrust value from controller, in quadrotor frame
      # M - ndarray, control moment from controller, in quadrotor frame
      # T - ndarray, tension value from simulation
          
      # OUTPUTS:
      # sdot   - 13 x 1, derivative of state vector s
      
      #************ EQUATIONS OF MOTION ************************
      # print("taut")
      # Assign states
      vel   = qd[3:6]
      quat  = qd[6:10]
      omega = qd[10:13]
      R = utilslib.QuatToRot(quat)

      # Acceleration
      #accel = 1 / self.uav_params.mass * (R * np.array([[0],[0],[F]]) + T * xi - np.array([[0],[0],[params.mass * params.grav]]))
      accel =  (np.matmul(R , np.array([0,0,F])) + T)/self.uav_params[uav_idx].mass - np.array([0,0,self.uav_params[uav_idx].grav])
      
      # quaternion derivative 
      qdot = utilslib.quat_dot(quat,omega)

      # Angular acceleration
      pqrdot = np.matmul(self.uav_params[uav_idx].invI, M - np.cross(omega,np.matmul(self.uav_params[uav_idx].I, omega)))

      # Assemble sdot
      sdot = np.zeros(self.uav_dim_num)
      sdot[0] = vel[0]
      sdot[1] = vel[1]
      sdot[2] = vel[2]
      sdot[3] = accel[0]
      sdot[4] = accel[1]
      sdot[5] = accel[2]
      sdot[6] = qdot[0]
      sdot[7] = qdot[1]
      sdot[8] = qdot[2]
      sdot[9] = qdot[3]
      sdot[10] = pqrdot[0]
      sdot[11] = pqrdot[1]
      sdot[12] = pqrdot[2]
      return sdot
    
  def slack_quadEOM_readonly(self, qd, F, M, uav_idx): 
      # DESCRIPTION:
      # QUADEOM_READONLY Solve quadrotor equation of motion when the cable is
      # slack.
      # quadEOM_readonly calculate the derivative of the state vector
      
      # INPUTS:
      # qd - struct, quadrotor parameters you want to pass in
      # F - ndarray, thrust value from controller, in quadrotor frame
      # M - ndarray, control moment from controller, in quadrotor frame
      
      # OUTPUTS:
      # sdot   - 13 x 1, derivative of state vector s
      
      #************ EQUATIONS OF MOTION ************************
      # print("slack")
      # Assign states
      vel   = qd[3:6]
      quat  = qd[6:10]
      omega = qd[10:13]
      R = utilslib.QuatToRot(quat)

      # Acceleration
      accel =  np.matmul(R , np.array([0,0,F]))/self.uav_params[uav_idx].mass - np.array([0,0,self.uav_params[uav_idx].grav])
      # Angular velocity
      qdot = utilslib.quat_dot(quat,omega)

      # Angular acceleration
      pqrdot = np.matmul(self.uav_params[uav_idx].invI , M - np.cross(omega, np.matmul(self.uav_params[uav_idx].I, omega)))

      # Assemble sdot
      sdot = np.zeros(self.uav_dim_num)
      sdot[0] = vel[0]
      sdot[1] = vel[1]
      sdot[2] = vel[2]
      sdot[3] = accel[0]
      sdot[4] = accel[1]
      sdot[5] = accel[2]
      sdot[6] = qdot[0]
      sdot[7] = qdot[1]
      sdot[8] = qdot[2]
      sdot[9] = qdot[3]
      sdot[10] = pqrdot[0]
      sdot[11] = pqrdot[1]
      sdot[12] = pqrdot[2]
      return sdot

  def cooperative_check_inelastic(self, x):
      # DESCRIPTION:
      # cooperative_check_inelastic check if any cables are slack or taut
      
      # INPUTS:
      # x     - ndarray, state of the multi-robot-payload system
      
      # OUTPUTS:
      # collision_condition   - self.nquad x 1 ndarray, True False value as collision condition
      # state assignment
      nquad = self.nquad
      rho_vec_list = self.pl_params.rho_vec_list
      pl_pos = x[0:3].reshape((3, 1))
      pl_vel = x[3:6].reshape((3, 1))
      pl_rot = utilslib.QuatToRot(x[6:10])
      # pl_rot = pl_rot.T
      attach_pos = pl_pos + pl_rot @ rho_vec_list
      attach_vel = pl_vel + pl_rot @ utilslib.vec2asym(x[10:13]) @ rho_vec_list
      robot_pos = x[13*np.arange(1, nquad + 1) + np.array([[0], [1], [2]])]
      robot_vel = x[13*np.arange(1, nquad + 1) + np.array([[3], [4], [5]])]
      xi = (attach_pos - robot_pos) / np.linalg.norm(attach_pos - robot_pos, 2, 0)
      
      # check collision condition
      # print("xi is")
      # print(xi)
      # print("relative vel")
      # print((attach_vel - robot_vel))
      # print("product is ")
      # print(xi * (attach_vel - robot_vel))
      positive_attach_robot_vel = (np.sum(xi * (attach_vel - robot_vel), 0) > 1e-3)
      taut_condition = self.istaut(attach_pos, robot_pos, self.pl_params.cable_length)
      collision_condition = np.empty((taut_condition.shape))
      for i in range(taut_condition.shape[0]):
        collision_condition[i] = positive_attach_robot_vel[i] and taut_condition[i]
      return collision_condition, xi

  def rigidbody_quad_inelastic_cable_collision(self, x, collision_idx):
      # DESCRIPTION:
      # rigidbody_quad_inelastic_cable_collision redistribute velocities between quadrotor
      # and the payload immediately after one or more cables switch from slack to taut condition
      
      # INPUTS:
      # x                   - ndarray, state of the multi-robot-payload system
      # collision_idx       - ndarray, self.nquad by 1, True False value denotes collision is happen between payload and which quadrotor
      
      # OUTPUTS:
      # collision_condition   - self.nquad x 1 ndarray, True False value as collision condition
      # state assignment

      xL = x[0:13].reshape((13, 1))
      xQs = x[13:].reshape((x.shape[0]-13, 1))
      nquad = self.nquad
      ntaut_quad = 0
      for i in range(nquad):
          if collision_idx[i] != 0:
              ntaut_quad += 1
      IL = self.pl_params.I
      mL = self.pl_params.mass
      
      rho_vec_idx_bool = np.repeat(collision_idx, 3)
      rho_vec_idx_int = np.zeros(int(np.sum(rho_vec_idx_bool)), dtype=int)

      index = 0
      for j in range(3*nquad):
          if rho_vec_idx_bool[j] == 1:
              rho_vec_idx_int[index] = j
              index = index + 1
      temp_rho_vec_asym_mat = self.pl_params.rho_vec_asym_mat[:, rho_vec_idx_int]
      rho_vec_asym_mat = temp_rho_vec_asym_mat   
      
      collision_idx_int = [] 
      for j in range(nquad):
          if collision_idx[j] == 1:
              collision_idx_int.append(j) 
      #temp_rho_vec_list = self.pl_params.rho_vec_list[:, utilslib.toint(collision_idx_int)]
      temp_rho_vec_list = self.pl_params.rho_vec_list[:, collision_idx_int]
      rho_vec_list = temp_rho_vec_list

      pl_rot = utilslib.QuatToRot(xL[6:10])
      pl_omg = xL[10:13].reshape(3, 1)
      pl_vel = xL[3:6].reshape(3, 1)

      attach_pos = xL[0:3].reshape(3, 1) + pl_rot @ rho_vec_list
      robot_state = np.zeros((13, nquad))
      for i in range(nquad):
        robot_state[:, i] = xQs[13*i:13*(i+1), 0]
      #robot_pos = robot_state[0:3, utilslib.toint(collision_idx_int)]
      #robot_vel = robot_state[3:6, utilslib.toint(collision_idx_int)]
      robot_pos = robot_state[0:3, collision_idx_int]
      robot_vel = robot_state[3:6, collision_idx_int]
      
      # state computation
      xi = (attach_pos - robot_pos) / np.linalg.norm(attach_pos - robot_pos, 2, 0)
      robot_vel_xi_proj = np.sum(xi * robot_vel, 0)
      xixiT_robot_vel = xi * robot_vel_xi_proj
      xi_vert_robot_vel = robot_vel - xixiT_robot_vel
      hatrho_rotL_xi = -np.transpose(pl_rot @ rho_vec_asym_mat) @ xi
      hatrho_index = (3 * ntaut_quad + 3) * np.arange(ntaut_quad) + np.array([[0],[1],[2]])
      hatrho_index = hatrho_index % hatrho_rotL_xi.shape[0]

      temp_hatrho_rotL_xi = np.zeros((hatrho_index.shape), dtype=float)
      for i in range(hatrho_index.shape[1]): # for each col of the index
          temp_hatrho_rotL_xi[:, i] = hatrho_rotL_xi[hatrho_index[:, i], i]
      hatrho_rotL_xi = temp_hatrho_rotL_xi

      # cal for collision
      G1 = np.vstack((xi, hatrho_rotL_xi))
      ML_top = np.hstack((mL * np.eye(3), np.zeros((3, 3))))
      ML_bottom = np.hstack((np.zeros((3, 3)), IL))
      ML = np.vstack((ML_top, ML_bottom))
      MQ_list = []
      for collide_uav_idx in collision_idx_int: 
        MQ_list.append(self.uav_params[collide_uav_idx].mass)
      #MQ = self.uav_params.mass * np.eye(ntaut_quad)
      MQ = np.diag(np.array(MQ_list)) 
      M = ML + G1 @ MQ @ G1.T
      right_element = xi @ MQ @ robot_vel_xi_proj.T
      right_element = right_element.reshape((right_element.shape[0],1))
      left_element = hatrho_rotL_xi @ MQ @ robot_vel_xi_proj.T
      left_element = left_element.reshape((left_element.shape[0], 1))
      
      b = np.vstack((mL*pl_vel+right_element, left_element + IL @ pl_omg))

      collided_pl_vel_omg,_,_,_ = np.linalg.lstsq(M,b)
      collided_pl_vel = collided_pl_vel_omg[0:3]
      collided_pl_omg = collided_pl_vel_omg[3:6]

      collided_robot_vel_proj = xi * sum(xi * (collided_pl_vel + pl_rot @ utilslib.vec2asym(collided_pl_omg) @ rho_vec_list), 0)
      collided_robot_vel = xi_vert_robot_vel + collided_robot_vel_proj

      # Return States
      x[3:6] = collided_pl_vel.reshape((collided_pl_vel.shape[0],))
      x[10:13] = collided_pl_omg.reshape((collided_pl_omg.shape[0],))
      counter = 0
      for nq in range(1, (nquad + 1)):
          if collision_idx[nq - 1] == 1:
              counter = counter + 1
              temp = collided_robot_vel[:,counter-1]
              temp = temp.reshape((temp.shape[0],1))
              x[13*nq + np.array([[3],[4],[5]])] = temp
      return x

####################################################################################
##################                    PTMASS                    ####################
####################################################################################
  def hybrid_ptmass_pl_transportationEOM(self, t, s):
    # DESCRIPTION:
    # QUADEOM Wrapper function for solving quadrotor equation of motion
    # 	quadEOM takes in time, state vector, and output the derivative of the state vector, the
    # 	actual calcution is done in quadEOM_readonly.
    #
    # INPUTS:
    # t             - 1 x 1, time
    # s computed    - 19 x 1, state vector = [xL, yL, zL, xLd, yLd, zLd,
    #                                         xQ, yQ, zQ, xQd, yQd, zQd,
    #                                         qw, qx, qy, qz, pQ, qQ, rQ]
    #
    # take_in/output- 26 x 1, state vector(dot-elementwise) = [ xL,     yL,     zL,     xLd,    yLd,    zLd,
    #                                                           qLw = 0,qLx = 0,qLy = 0,qLz = 0,pL = 0, qL = 0, rL = 0,
    #                                                           xQ,     yQ,     zQ,     xQd,    yQd,    zQd
    #                                                           qw,     qx,     qy,     qz,     pQuad,  qQuad,  rQuad]
    #
    #                   ,where [xL, yL, zL] is payload postion
    #                          [xLd, yLd, zLd], is payload linear velocity
    #                          [xQ, yQ, zQ] is quadrotor position
    #                          [xQd, yQd, zQd] is quadrotor velocity
    #                          [qw, qx, qy, qz] is quadrotor orientation in quaternion
    #                          [pQ, qQ, rQ] is quadrotor angular velocity in its own frame
    # OUTPUTS:
    # sdot          - 26 x 1, derivative of state vector s as mentioned above (some values are forced to 0)
      l = self.pl_params.cable_length
      plqd = {}
      # convert state s to plqd
      plqd["pos"] = s[0:3]
      plqd["vel"] = s[3:6]
      plqd["qd_pos"] = s[13:16]
      plqd["qd_vel"] = s[16:19]
      plqd["qd_quat"] = s[19:23]
      plqd["qd_omega"] = s[23:26]
      Rot = utilslib.QuatToRot(s[19:23])
      plqd["qd_rot"] = Rot
      quad_load_rel_pos = plqd["qd_pos"]-plqd["pos"]
      quad_load_rel_vel = plqd["qd_vel"]-plqd["vel"]

      if self.cable_is_slack[0]:   
          return self.slack_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F, self.uav_M)
      else:
          plqd["xi"] = -quad_load_rel_pos/l
          plqd["xidot"] = -quad_load_rel_vel/l
          return self.taut_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F, self.uav_M)

  def slack_ptmass_payload_quadEOM_readonly(self, t, plqd, F, M):
      # DESCRIPTION:
      # slack_ptmass_payload_quadEOM_readonly Solve payload equation of motion
      # calculating the derivative of the state vector when cable is slack
          
      # INPUTS:
      # plqd - struct, quadrotor and payload parameters you want to pass in
      # F - ndarray, thrust value from controller, in quadrotor frame
      # M - ndarray, control moment from controller, in quadrotor frame
      # t - float, time, not used currently
          
      # OUTPUTS:
      # sdot   - 26 x 1, derivative of state vector s
      # # Assign params and states
      mQ  =   self.uav_params[0].mass
      e3  =   np.array([[0.0],[0.0],[1.0]])
      g   =   self.uav_params[0].grav * e3
      wRb =   plqd["qd_rot"];   # Rotation matrix of the quadrotor
      qd_quat     =   plqd["qd_quat"]
      qd_omega    =   plqd["qd_omega"]
      p = qd_omega[0]
      q = qd_omega[1]
      r = qd_omega[2]

      # Obtain Quadrotor Force Vector
      quad_force_vector = F * wRb @ e3 

      # Solving for Quadrotor Acceleration
      accQ = quad_force_vector/mQ - g; 

      # Solving for Quadrotor Angular Velocity
      K_quat = 2.0 # this enforces the magnitude 1 constraint for the quaternion
      quaterror = 1 - np.linalg.norm(qd_quat)
      qdot =  1/2*np.array([[0, -p, -q, -r],
                            [p,  0,  r, -q],
                            [q, -r,  0,  p],
                            [r,  q, -p,  0]]) @ qd_quat.reshape((qd_quat.shape[0], 1)) + K_quat * quaterror * qd_quat.reshape((qd_quat.shape[0], 1))

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params[0].invI @ (M - np.reshape(np.cross(qd_omega, self.uav_params[0].I @ qd_omega, axisa=0, axisb=0), (3,1)))
      
      # Assemble sdot
      sdot = np.zeros((26,1), dtype=float)
      sdot[0:3] = plqd["vel"].reshape(3, 1)
      sdot[3:6] = -g
      sdot[13:16] = plqd["qd_vel"].reshape(3, 1)
      sdot[16:19] = accQ
      sdot[19:23] = qdot
      sdot[23:26] = pqrdot
      sdot = sdot[:,0]
      return sdot

  def taut_ptmass_payload_quadEOM_readonly(self, t, plqd, F, M):
      # DESCRIPTION:
      # taut_ptmass_payload_quadEOM_readonly Solve payload equation of motion
      # calculating the derivative of the state vector when cable is taut
          
      # INPUTS:
      # plqd - struct, quadrotor and payload parameters you want to pass in
      # F - ndarray, thrust value from controller, in quadrotor frame
      # M - ndarray, control moment from controller, in quadrotor frame
      # t - float, time, not used currently
          
      # OUTPUTS:
      # sdot   - 26 x 1, derivative of state vector s
      mL = self.pl_params.mass
      mQ = self.uav_params[0].mass
      total_mass = mL + mQ
      l = self.pl_params.cable_length
      xi = plqd["xi"]
      xidot = plqd["xidot"]
      xi_omega = np.cross(xi,xidot)
      e3=np.array([[0.0],[0.0],[1.0]])
      g = self.uav_params[0].grav * e3
      wRb=plqd["qd_rot"]    #Rotation matrix of the quadrotor
      qd_quat = plqd["qd_quat"]
      qd_omega = plqd["qd_omega"]
      p = qd_omega[0]
      q = qd_omega[1]
      r = qd_omega[2]

      # Obtain tension vector
      quad_force_vector = F * wRb @ e3
      quad_centrifugal_f = mQ * l * (xi_omega.T @ xi_omega)
      tension_vector = mL * (-xi.T.reshape(1,3) @ quad_force_vector + quad_centrifugal_f) * xi.reshape(3,1) / total_mass
      # Solving for Load Acceleration
      accL = - tension_vector / mL - g
      # Solving for Quadrotor Acceleration
      accQ = (quad_force_vector + tension_vector) / mQ - g

      # Solving for Quadrotor Angular Velocity
      K_quat = 2 # this enforces the magnitude 1 constraint for the quaternion
      quaterror = 1 - np.linalg.norm(qd_quat)
      qdot =  1/2*np.array([[0, -p, -q, -r],
                            [p,  0,  r, -q],
                            [q, -r,  0,  p],
                            [r,  q, -p,  0]]) @ qd_quat.reshape((qd_quat.shape[0], 1)) + K_quat * quaterror * qd_quat.reshape((qd_quat.shape[0], 1))

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params[0].invI @ (M - np.cross(qd_omega, self.uav_params[0].I @ qd_omega, axisa=0, axisb=0).T.reshape(3, 1))

      # Assemble sdot
      sdot = np.zeros((26,1), dtype=float)
      sdot[0:3] = plqd["vel"].reshape(3, 1)
      sdot[3:6] = accL
      sdot[13:16] = plqd["qd_vel"].reshape(3, 1)
      sdot[16:19] = accQ
      sdot[19:23] = qdot
      sdot[23:26] = pqrdot
      sdot = sdot[:,0]

      return sdot

  def ptmass_inelastic_cable_collision(self, x1, x2, m1, m2): 
      # DESCRIPTION:
      # ptmass_inelastic_cable_collision 
      # redistribute the velocity between payload and the quadrotor by assuming
      # perfectly inelastic collision of ptmass and the drone along the cable direction
          
      # INPUTS:
      # x1 - float, position of object 1
      # x2 - float, position of object 2
      # m1 - float, mass of object 1
      # m2 - float, mass of object 2
          
      # OUTPUTS:
      # v1, v2   - both float, new velocities for the two object
      
      obj1_pos = x1[0:3]
      obj1_pos = obj1_pos.reshape((3, 1))
      obj2_pos = x2[0:3]
      obj2_pos = obj2_pos.reshape((3, 1))
      obj1_vel = x1[3:6]
      obj2_pos = obj2_pos.reshape((3, 1))
      obj2_vel = x2[3:6]
      obj2_pos = obj2_pos.reshape((3, 1))

      cable_direction = (obj2_pos - obj1_pos) / np.linalg.norm(obj2_pos - obj1_pos)
      cable_direction = cable_direction.reshape((3, 1))
      cable_direction_projmat = cable_direction @ cable_direction.T
      v1_proj = cable_direction_projmat @ obj1_vel
      v2_proj = cable_direction_projmat @ obj2_vel
      
      v = (m1 * v1_proj + m2 * v2_proj)/(m1+m2)
      
      v1 = v + obj1_vel - v1_proj
      v2 = v + obj2_vel - v2_proj

      return v1, v2

####################################################################################
##################                  Rigidlink                   ####################
####################################################################################
  def rigid_links_cooperative_rigidbody_pl_EOM(self, t, s):
    # DESCRIPTION
    # QUADEOM Wrapper function for solving quadrotor equation of motion
    # 	quadEOM takes in time, state vector, and output the derivative of the state vector, the
    # 	actual calcution is done in quadEOM_readonly.
    #
    # INPUTS:
    # t             - 1 x 1, time
    # s: take_in    - 26 x 1, state vector(dot-elementwise) = [ xL,         yL,         zL,     
    #                                                           xLd,        yLd,        zLd,
    #                                                           qLw = 0,    qLx = 0,    qLy = 0,    qLz = 0,    
    #                                                           pL,         qL,         rL,
    #                                                           xQ = 0,     yQ = 0,     zQ = 0, 
    #                                                           xQd = 0,    yQd = 0,    zQd = 0,
    #                                                           qw,         qx,         qy,         qz,     
    #                                                           pQuad = 0,  qQuad = 0,  rQuad = 0]
    #
    #                   ,where [xL, yL, zL] is payload postion
    #                          [xLd, yLd, zLd], is payload linear velocity
    #                          [qLw, qLx, qLy, qLz], is payload orientation in quaternion
    #                          [pL, qL, rL], is payload orientation in euler angle
    #                          [xQ, yQ, zQ] is quadrotor position
    #                          [xQd, yQd, zQd] is quadrotor velocity
    #                          [qw, qx, qy, qz] is quadrotor orientation in quaternion
    #                          [pQuad, qQuad, rQuad] is quadrotor angular velocity in its own frame
    # OUTPUTS:
    # sdot          - 26 x 1, derivative of state vector s as mentioned above (some values are forced to 0)
  
      qd = {}
      # extract payload state_vector
      qd["pos"] = s[0:3]
      qd["vel"] = s[3:6]
      qd["quat"] = s[19:23]
      qd["omega"] = s[10:13]
      qd["rot"] = utilslib.QuatToRot(qd["quat"])

      result = self.rigidbody_structEOM_readonly(t, qd, self.uav_F, self.uav_M)
      return result
  
  def rigid_links_cooperative_payload_controller_init(self, ql, params):
    # DESCRIPTION:
    # rigid_links_cooperative_payload_controller_init() initialize the robot_thrust_moment for rigid link robot system
    # this function is only called once when initialization happens at __init__(), this is the same as the function in controller.py
    #
    # INPUTS:
    # ql - struct, quadrotor and payload parameters you want to pass in
    # params - struct, store important simulation parameter
    #
    # OUTPUTS:
    # u          - ndarray, 6*1, controlled force and moment
        if not params.sim_start:
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
        ep = ep.reshape((3,1))
        ed = ed.reshape((3,1))
        # Desired acceleration This equation drives the errors of trajectory to zero.
        acceleration_des = ql["acc_des"] + params.Kp @ ep + params.Kd @ ed

        # Net force F=kx*ex kv*ex_dot + mge3 +mxdes_ddot
        Force = m*g*e3  + m*acceleration_des

        tau = np.transpose(Force) @ Rot @ e3

        Rot_des = np.zeros((3,3), dtype=float)
        Z_body_in_world = Force/np.linalg.norm(Force)
        Rot_des[:,2:3] = Z_body_in_world
        X_unit = np.array([[np.cos(yaw_des)], [np.sin(yaw_des)], [0]])
        Y_body_in_world = np.cross(Z_body_in_world,X_unit, axisa=0, axisb=0).T
        Y_body_in_world = Y_body_in_world/np.linalg.norm(Y_body_in_world)
        Rot_des[:,1:2] = Y_body_in_world
        X_body_in_world = np.cross(Y_body_in_world,Z_body_in_world, axisa=0, axisb=0).T
        Rot_des[:,0:1] = X_body_in_world

        ## Attitude Control

        # Errors of anlges and angular velocities

        e_Rot = np.transpose(Rot_des) @ Rot - np.transpose(Rot) @ Rot_des
        e_angle = vee(e_Rot)/2
        e_omega = omega.reshape((3,1)) - np.transpose(Rot) @ Rot_des @ omega_des.reshape((3, 1))

        # Net moment
        # Missing the angular acceleration term but in general it is neglectable.
        test = params.struct_I @ omega
        M = - params.Kpe @ e_angle - params.Kde @ e_omega + np.cross(omega, params.struct_I @ omega, axisa=0, axisb=0).reshape((3,1))
        ## Quadrotor Thrust and Moment Distribution
        u = params.thrust_moment_distribution_mat @ np.vstack((tau, M))

        return u

  def rigidbody_structEOM_readonly(self, t, s, F, M):
    # DESCRIPTION:
    # rigidbody_structEOM_readonly() Solve payload equation of motion
    # calculating the derivative of the state vector
    #
    # INPUTS:
    # s - struct, state of the rigidlink system
    # F - ndarray, thrust value from controller, in quadrotor frame
    # M - ndarray, control moment from controller, in quadrotor frame
    # t - float, time, not used currently
    #
    # OUTPUTS:
    # u          - ndarray, 6*1, controlled force and moment
      # Assign states
      quat = s["quat"]
      qW = quat[0]
      qX = quat[1]
      qY = quat[2]
      qZ = quat[3]
      omega = s["omega"]
      p = omega[0]
      q = omega[1]
      r = omega[2]

      # Payload quaternion first derivative
      K_quat = 2; #this enforces the magnitude 1 constraint for the quaternion
      quaterror = 1 - (qW**2 + qX**2 + qY**2 + qZ**2)
      qLdot =  1/2*np.array([   [0, -p, -q, -r],
                                [p,  0,  r, -q],
                                [q, -r,  0,  p],
                                [r,  q, -p,  0]]) @ quat + K_quat * quaterror * quat

      # Payload Angular acceleration
      # all uav orientations are the same
      # pick first uav orientation
      omgLdot = M - np.cross(omega,self.pl_params.struct_I @ s["omega"], axisa=0, axisb=0).T.reshape((3,1))

      # Payload Acceleration
      # combine robot's forces
      accL = F/self.pl_params.struct_mass - self.pl_params.grav*np.array([[0],[0],[1]])
      # print(accL)
      # Assemble sdot
      # original          - 13 x 1, state vector = [xL,   yL,     zL, 
      #                                             xLd,  yLd,    zLd, 
      #                                             qw,   qx,     qy,     qz,   
      #                                             pL,   qL,     rL]
      # s: take_in/output - 26 x 1, state vector(dot-elementwise) = [ xL,         yL,         zL,     
      #                                                               xLd,        yLd,        zLd,
      #                                                               qLw = 0,    qLx = 0,    qLy = 0,    qLz = 0,    
      #                                                               pL,         qL,         rL,
      #                                                               xQ = 0,     yQ = 0,     zQ = 0, 
      #                                                               xQd = 0,    yQd = 0,    zQd = 0,
      #                                                               qw,         qx,         qy,         qz,     
      #                                                               pQuad = 0,  qQuad = 0,  rQuad = 0]
      sdotLoad = np.zeros((26,1), dtype=float)

      sdotLoad[0:3,0] = s["vel"]
      sdotLoad[3,0] = accL[0, 0]
      sdotLoad[4,0] = accL[1, 0]
      sdotLoad[5,0] = accL[2, 0]

      sdotLoad[6,0] = qLdot[0]
      sdotLoad[7,0] = qLdot[1]
      sdotLoad[8,0] = qLdot[2]
      sdotLoad[9,0] = qLdot[3]

      sdotLoad[10,0] = omgLdot[0]
      sdotLoad[11,0] = omgLdot[1]
      sdotLoad[12,0] = omgLdot[2]
      
      sdotLoad[13:26, 0 ] = sdotLoad[0:13,0]

      sdotLoad = sdotLoad.reshape((26, ))
      return sdotLoad

##################                  Call backs                  ####################
  
  def ext_wrench_callback(self, wrench_command):
      self.ext_force = np.array([wrench_command.force.x, wrench_command.force.y, wrench_command.force.z])
      self.ext_torque= np.array([wrench_command.torque.x, wrench_command.torque.y, wrench_command.torque.z])

  def rpm_command_callback(self,rpm_command,uav_id):
      return

  def fm_command_callback(self,fm_command,uav_id):
    # DESCRIPTION:
    # call back function for fm_command, get force and moment command from quadrotor controllers
        if self.pl_params.mechanism_type == 'Rigid Link':
            self.uav_F[0,0] = fm_command.rlink_thrust.x
            self.uav_F[1,0] = fm_command.rlink_thrust.y
            self.uav_F[2,0] = fm_command.rlink_thrust.z

            self.uav_M[0,0] = fm_command.moments.x
            self.uav_M[1,0] = fm_command.moments.y
            self.uav_M[2,0] = fm_command.moments.z
        elif self.pl_params.mechanism_type == 'Cable':
            if self.pl_params.payload_type == 'Rigid Body':
                self.uav_F[uav_id] = fm_command.thrust
                self.uav_M[0,uav_id] = fm_command.moments.x
                self.uav_M[1,uav_id] = fm_command.moments.y
                self.uav_M[2,uav_id] = fm_command.moments.z
            elif self.pl_params.payload_type == 'Point Mass':
                self.uav_F[0] = fm_command.thrust
                self.uav_M[0,0] = fm_command.moments.x
                self.uav_M[1,0] = fm_command.moments.y
                self.uav_M[2,0] = fm_command.moments.z
