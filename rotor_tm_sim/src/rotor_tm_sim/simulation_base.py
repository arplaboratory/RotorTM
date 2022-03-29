import numpy as np
import numpy.linalg as linalg
from yaml.error import Mark
import rospy
import scipy.integrate
from scipy.spatial.transform import Rotation as rot_math
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry 
from rotor_tm_msgs.msg import RPMCommand, FMCommand 
from rotor_tm_utils import utilslib, rosutilslib
from rotor_tm_utils.vee import vee
from rotor_tm_utils.utilslib import vecnorm
import time


def ptmassslackToTaut(t, x):
    value = np.linalg.norm(x[0:3] - x[13:16]) - ptmassslackToTaut.cable_length
    # print("I'm in evnet ", value)
    return value

def ptmasstautToSlack(t, x):
    value = np.linalg.norm(x[0:3] - x[13:16]) - ptmasstautToSlack.cable_length + 0.001
    return value


class simulation_base():
  def __init__(self,pl_params,uav_params):
      rospy.init_node('simulation')
      self.rate = 100
      rate = rospy.Rate(self.rate) # 100hz
      t_span = (0,1/self.rate)
      self.worldframe = "simulator"

      # init parameters
      self.pl_params = pl_params
      self.uav_params = uav_params
      self.nquad = self.pl_params.nquad
      self.pl_dim_num = 13
      self.uav_dim_num = 13
      self.mav_name = 'dragonfly'
      self.sim_start = False
      self.last_odom_time_received = 0.0

      if self.nquad != 1:
        if self.pl_params.id == "Cable":
            self.uav_F = np.matmul(self.pl_params.pseudo_inv_P, np.array([0,0,self.pl_params.mass * self.pl_params.grav,0,0,0])) + np.kron([1]*self.nquad, [0,0,self.uav_params.mass * self.pl_params.grav]) 
            # self.uav_F = self.uav_F.reshape(3,self.nquad)[:,2]
            self.uav_F = self.uav_F.reshape(self.nquad, 3)[:,2]
            self.uav_M = np.zeros((3,self.nquad))
            self.cable_is_slack = np.zeros(self.nquad)

            self.rho_vec_list = self.pl_params.rho_vec_list
            self.cable_len_list = np.array(self.pl_params.cable_length)

            x = np.zeros(self.pl_dim_num + self.uav_dim_num * self.nquad)
            for i in range(self.nquad+1): 
                if i > 0:
                    print("initalizing robot ", i)
                    x[i*13:i*13+3] = x[0:3] + self.rho_vec_list[:,i-1] + np.array([0,0,self.cable_len_list[i-1]])
                x[i*13 + 6] = 1
        else: 
            ## init for rigidlink
            ## init force to [sum of (quad and payload mass)] * gravity
            ## init M to [0; 0; 0]
            ## init calbe to taut
            print("Initalizing payload-robot rigidlink structure")
            # weight = self.pl_params.struct_mass * self.uav_params.grav
            self.cable_is_slack = np.zeros(self.nquad)
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

      else:
        ## init for ptmass
        ## init force to [sum of (quad and payload mass)] * gravity
        ## init M to [0; 0; 0]
        ## init calbe to taut
        ## init rho_vec_list = [] (empty)
        ## init cable_len_list = cable_length (read)
        ## init state x as a (26, 1) state vector with inital position hard code to (0, 0, 0.5)
        print("Initalizing ptmass robot")
        self.uav_F = np.array([(self.pl_params.mass + self.uav_params.mass) * self.uav_params.grav])
        self.uav_M = np.zeros((3,self.nquad), dtype=float)
        self.cable_is_slack = np.zeros(self.nquad)
        self.rho_vec_list = self.pl_params.rho_vec_list
        self.cable_len_list = np.array(self.pl_params.cable_length)
        
        # x = (26, 1) state init
                                                    # Name      Element Location
        x = np.array([0.0,  0.0,    0.0,            # pl pos    3
                      0.0,  0.0,    0.0,            # pl vel    6
                      1.0,  0.0,    0.0,    0.0,    # pl quat   10
                      0.0,  0.0,    0.0,            # pl omega  13
                      0.0,  0.0,    0.0,            # qd pos    16
                      0.0,  0.0,    0.0,            # qd vel    19 
                      1.0,  0.0,    0.0,    0.0,    # qd quat   23
                      0.0,  0.0,    0.0])           # qd omega  26


      # ROS Publisher 
      self.system_publisher = rospy.Publisher('system/marker',MarkerArray,queue_size=10)
      self.payload_odom_publisher = rospy.Publisher('payload/odom',Odometry,queue_size=1, tcp_nodelay=True)
      self.robot_odom_publisher = []
      self.attach_publisher = []
      for i in range(self.nquad):
          self.robot_odom_publisher.append(rospy.Publisher(self.mav_name + str(i+1) + '/odom',Odometry, queue_size=1, tcp_nodelay=True))
          self.attach_publisher.append(rospy.Publisher(self.mav_name + str(i+1) + '/attach',Odometry, queue_size=1, tcp_nodelay=True))
           
      # ROS Subscriber 
      self.robot_command_subscriber = []
      for uav_id in range(self.nquad):
          mav_name = self.mav_name + str(uav_id+1)
          controller_name = "/controller_" + str(uav_id+1)
          self.robot_command_subscriber.append(rospy.Subscriber(controller_name + '/' + mav_name + '/rpm_cmd',RPMCommand,self.rpm_command_callback,uav_id,queue_size=1, tcp_nodelay=True))
          self.robot_command_subscriber.append(rospy.Subscriber(controller_name + '/' + mav_name + '/fm_cmd',FMCommand,self.fm_command_callback,uav_id,queue_size=1, tcp_nodelay=True))
    
      # Visualization Init
      self.cable_marker_scale = 0.01 * np.ones(3)
      self.cable_marker_color = np.array([1.0,0.5,0.5,0.5])
      self.uav_marker_scale = 0.5 * np.ones(3)
      self.uav_marker_color = np.array([1.0,0.0,0.0,1.0])
      self.uav_mesh = self.uav_params.mesh_path
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

      while not rospy.is_shutdown():
        # Without event
        #tsave, xsave = scipy.integrate.solve_ivp(rotortm_simulation_base.run(), t_span, x, method='RK45', event = simulation_base.guard())
        # With event
        start = time.time()
        if self.pl_params.id == "Rigid Link":
            ## this is rigid link sim
            sol = scipy.integrate.solve_ivp(self.rigid_links_cooperative_rigidbody_pl_EOM, t_span, x, method= 'RK45', t_eval=t_span)
        else:
            if self.nquad == 1:
                ## this is point mass sim
                ## inelastic collision
                pl_pos = x[0:3]
                pl_vel = x[3:6]
                robot_pos = x[13:16]
                robot_vel = x[16:19]
                cable_norm_vel = np.transpose(pl_pos - robot_pos) @ (pl_vel - robot_vel)
                if cable_norm_vel > 1e-6 and not self.cable_is_slack:
                    v1, v2 = self.ptmass_inelastic_cable_collision(x[0:6], x[13:19], self.pl_params.mass, self.uav_params.mass)
                    x[3:6] = v1
                    x[16:19] = v2
                
                ## set up event
                ptmasstautToSlack.terminal = True
                ptmassslackToTaut.terminal = True
                ptmasstautToSlack.direction = -1
                ptmassslackToTaut.direction = 1
                ptmassslackToTaut.cable_length = self.pl_params.cable_length
                ptmasstautToSlack.cable_length = self.pl_params.cable_length

                if self.cable_is_slack:
                    print("Cable is slack")
                    sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmassslackToTaut)
                else:
                    print("Cable is taut")
                    sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span, events=ptmasstautToSlack)
            else:    
                sol = scipy.integrate.solve_ivp(self.hybrid_cooperative_rigidbody_pl_transportationEOM, t_span, x, method='RK23', t_eval=t_span)
        end = time.time()
        # print(sol.y)
        x = sol.y[:,-1]
        # print(x)
        # print(np.linalg.norm(x[0:3] - x[13:16]), (self.pl_params.cable_length - 1e-1))
        self.cable_is_slack = self.isslack(x[0:3], x[13:16], self.pl_params.cable_length)

        # The simulation must first run on the quadrotors
        # Then the simulation simulates the dynamics of the payload
        # If this is the first iteration in the simulation,
        # initialize the plots of the quads
        # The integration stops when the distance between robot and payload
        # equals to the cable length
        '''
        inelastic_collision_flag = check_inelastic_collision_idx(x,pl_params)
 
        while np.any(inelastic_collision_flag):
 
            print('collide!\n' % ())
            before_collide_inelastic_collision_flag = inelastic_collision_flag
            x = rigidbody_quad_inelastic_cable_collision(x,pl_params,quad_params,inelastic_collision_flag)
            print('after collide!\n' % ())
            after_collide_inelastic_collision_flag = check_inelastic_collision_idx(x,pl_params)
            if np.any((after_collide_inelastic_collision_flag - before_collide_inelastic_collision_flag) > 0):
                inelastic_collision_flag = np.logical_or(after_collide_inelastic_collision_flag,before_collide_inelastic_collision_flag)
            else:
                inelastic_collision_flag = after_collide_inelastic_collision_flag
 
        # State Integration
        slack_condition = cable_is_slack
        option = odeset('Events',lambda t = None,x = None: cooperativeGuard(t,x,pl_params,slack_condition))
        tsave,xsave = ode45(lambda t = None,s = None: dynamicshandle(t,s,pl_params),timeint,x,option)
        x = np.transpose(xsave(end(),:))
        # Check if the cable is slack
        cable_is_slack = not istaut(x(np.arange(1,3+1)) + np.transpose(QuatToRot(x(np.arange(7,10+1)))) * pl_params.rho_vec_list,x(13 * np.array([np.arange(1,nquad+1)]) + np.array([[1],[2],[3]])),pl_params.cable_length) 
        '''
        
        # Publish payload odometry
        current_time = rospy.get_rostime()
        payload_odom = Odometry()
        payload_odom.header.stamp = current_time
        payload_odom.header.frame_id = self.worldframe 
        payload_rotmat = utilslib.QuatToRot(sol.y[:,0][6:10])

        if self.pl_params.mechanism_type == 'Rigid Link':
            self.load_pos = x[0:3].reshape((3,1)) + payload_rotmat @ self.pl_params.rho_load
            payload_odom.pose.pose.position.x = self.load_pos[0]
            payload_odom.pose.pose.position.y = self.load_pos[1]
            payload_odom.pose.pose.position.z = self.load_pos[2]
        else:
            payload_odom.pose.pose.position.x    = x[0]
            payload_odom.pose.pose.position.y    = x[1]
            payload_odom.pose.pose.position.z    = x[2]
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
        self.payload_odom_publisher.publish(payload_odom)


        system_marker = MarkerArray()
        cable_point_list = np.zeros((2*self.nquad,3))
        for uav_id in range(self.nquad):
            if self.pl_params.mechanism_type == 'Rigid Link':
                uav_state = x[13:26]
                attach_pos = self.load_pos.reshape((3,)) + np.matmul(payload_rotmat, (self.pl_params.rho_robot[:,uav_id]+np.array([0.028,0,0.032])))
                # attach_pos = x[0:3] + np.matmul(payload_rotmat, self.pl_params.rho_robot[:,uav_id])
                uav_state[0:3] = attach_pos
                attach_vel = uav_state[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.pl_params.rho_robot[:,uav_id]))
                #print("old norm is", np.linalg.norm(uav_state[0:3] - attach_pos[0:3]))
                if not self.cable_is_slack[uav_id]:
                    uav_attach_vector = uav_state[0:3] - attach_pos[0:3]
                    uav_attach_distance = np.linalg.norm(uav_attach_vector)
                    if uav_attach_distance > self.cable_len_list[uav_id]:
                        xi = uav_attach_vector/uav_attach_distance
                        uav_state[0:3] = attach_pos[0:3] + self.cable_len_list[uav_id] * xi
                        #print(uav_state[0:3])
                        #print(x[self.pl_dim_num+self.uav_dim_num*uav_id:self.pl_dim_num+self.uav_dim_num*(uav_id+1)])
                        #print("new norm is", np.linalg.norm(uav_state[0:3] - attach_pos[0:3]))
                    
                # Publish UAV odometry
                uav_odom = Odometry()
                uav_odom.header.stamp = current_time
                uav_odom.header.frame_id = self.worldframe 
                uav_odom.pose.pose.position.x = uav_state[0]
                uav_odom.pose.pose.position.y = uav_state[1]
                uav_odom.pose.pose.position.z = uav_state[2]
                uav_odom.twist.twist.linear.x = uav_state[3]
                uav_odom.twist.twist.linear.y = uav_state[4]
                uav_odom.twist.twist.linear.z = uav_state[5]
                uav_odom.pose.pose.orientation.w = uav_state[6]
                uav_odom.pose.pose.orientation.x = uav_state[7]
                uav_odom.pose.pose.orientation.y = uav_state[8]
                uav_odom.pose.pose.orientation.z = uav_state[9]
                uav_odom.twist.twist.angular.x = uav_state[10]
                uav_odom.twist.twist.angular.y = uav_state[11]
                uav_odom.twist.twist.angular.z = uav_state[12]
                self.robot_odom_publisher[uav_id].publish(uav_odom)

                # Publish UAV attach odometry
                attach_odom = Odometry()
                attach_odom.header.stamp = current_time
                attach_odom.header.frame_id = self.worldframe 
                # TODO: continue here publish the attach odom. 
                # attach_pos = x[0:3] + np.matmul(payload_rotmat, self.rho_vec_list[:,uav_id])
                # attach_vel = x[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.rho_vec_list[:,uav_id]))
                attach_odom.pose.pose.position.x = attach_pos[0]
                attach_odom.pose.pose.position.y = attach_pos[1]
                attach_odom.pose.pose.position.z = attach_pos[2]
                attach_odom.twist.twist.linear.x = attach_vel[0]
                attach_odom.twist.twist.linear.y = attach_vel[1]
                attach_odom.twist.twist.linear.z = attach_vel[2]
                self.attach_publisher[uav_id].publish(attach_odom)
            else:
                uav_state = x[self.pl_dim_num+self.uav_dim_num*uav_id:self.pl_dim_num+self.uav_dim_num*(uav_id+1)]
                attach_pos = x[0:3] + np.matmul(payload_rotmat, self.rho_vec_list[:,uav_id])
                attach_vel = x[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.rho_vec_list[:,uav_id]))
                #print("old norm is", np.linalg.norm(uav_state[0:3] - attach_pos[0:3]))
                if not self.cable_is_slack[uav_id]:
                    uav_attach_vector = uav_state[0:3] - attach_pos[0:3]
                    uav_attach_distance = np.linalg.norm(uav_attach_vector)
                    if uav_attach_distance > self.cable_len_list[uav_id]:
                        xi = uav_attach_vector/uav_attach_distance
                        uav_state[0:3] = attach_pos[0:3] + self.cable_len_list[uav_id] * xi
                        #print(uav_state[0:3])
                        #print(x[self.pl_dim_num+self.uav_dim_num*uav_id:self.pl_dim_num+self.uav_dim_num*(uav_id+1)])
                        #print("new norm is", np.linalg.norm(uav_state[0:3] - attach_pos[0:3]))
                    
                # Publish UAV odometry
                uav_odom = Odometry()
                uav_odom.header.stamp = current_time
                uav_odom.header.frame_id = self.worldframe 
                uav_odom.pose.pose.position.x = uav_state[0]
                uav_odom.pose.pose.position.y = uav_state[1]
                uav_odom.pose.pose.position.z = uav_state[2]
                uav_odom.twist.twist.linear.x = uav_state[3]
                uav_odom.twist.twist.linear.y = uav_state[4]
                uav_odom.twist.twist.linear.z = uav_state[5]
                uav_odom.pose.pose.orientation.w = uav_state[6]
                uav_odom.pose.pose.orientation.x = uav_state[7]
                uav_odom.pose.pose.orientation.y = uav_state[8]
                uav_odom.pose.pose.orientation.z = uav_state[9]
                uav_odom.twist.twist.angular.x = uav_state[10]
                uav_odom.twist.twist.angular.y = uav_state[11]
                uav_odom.twist.twist.angular.z = uav_state[12]
                self.robot_odom_publisher[uav_id].publish(uav_odom)
                #current_odom_time_received = rospy.get_time()
                #print("The current uav id is", uav_id)
                #print("The uav_odom publish time gap is", current_odom_time_received - self.last_odom_time_received)
                #self.last_odom_time_received = current_odom_time_received

                # Publish UAV attach odometry
                attach_odom = Odometry()
                attach_odom.header.stamp = current_time
                attach_odom.header.frame_id = self.worldframe 
                # TODO: continue here publish the attach odom. 
                # attach_pos = x[0:3] + np.matmul(payload_rotmat, self.rho_vec_list[:,uav_id])
                # attach_vel = x[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.rho_vec_list[:,uav_id]))
                attach_odom.pose.pose.position.x = attach_pos[0]
                attach_odom.pose.pose.position.y = attach_pos[1]
                attach_odom.pose.pose.position.z = attach_pos[2]
                attach_odom.twist.twist.linear.x = attach_vel[0]
                attach_odom.twist.twist.linear.y = attach_vel[1]
                attach_odom.twist.twist.linear.z = attach_vel[2]
                self.attach_publisher[uav_id].publish(attach_odom)
            cable_point_list[2*uav_id,:] = uav_state[0:3]
            cable_point_list[2*uav_id+1,:] = attach_pos[0:3]
            uav_marker_msg = rosutilslib.init_marker_msg(Marker(),10,0,self.worldframe,self.uav_marker_scale,self.uav_marker_color,self.uav_mesh)
            uav_marker = rosutilslib.update_marker_msg(uav_marker_msg,uav_state[0:3],uav_state[6:10],uav_id)
            system_marker.markers.append(uav_marker)

        # Update cable visualization
        cable_marker_msg = rosutilslib.init_marker_msg(Marker(),5,0,self.worldframe,self.cable_marker_scale,self.cable_marker_color)
        system_marker.markers.append(rosutilslib.update_line_msg(cable_marker_msg,cable_point_list,uav_id + 1))
        # Update payload visualization
        system_marker.markers.append(rosutilslib.update_marker_msg(self.payload_marker_msg,x[0:3],x[6:10],uav_id+2))
        self.system_publisher.publish(system_marker)

        rate.sleep()    


  def istaut(self, robot_pos, attach_pos, cable_length):
      if (np.linalg.norm(robot_pos - attach_pos) > (cable_length - 1e-1)):
        return np.array([1.0])
      else:
        return np.array([0.0])

  def isslack(self, robot_pos, attach_pos, cable_length):
      if (np.linalg.norm(robot_pos - attach_pos) > (cable_length - 1e-1)):
        return np.array([0.0])
      else:
        return np.array([1.0])

####################################################################################
##################                    hybrid                    ####################
####################################################################################
  def hybrid_cooperative_rigidbody_pl_transportationEOM(self, t, s): 
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
      self.attach_accel = self.pl_accel + np.array([0,0,self.uav_params.grav]) + \
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
              attach_qn_force = u_parallel - self.uav_params.mass * cable_len * linalg.norm(cbl_omg)**2 * xi - \
                                self.uav_params.mass * np.matmul(xixiT, np.matmul(pl_rot, attach_centrifugal_accel[:,uav_idx]))
              attach_qn_moment = np.matmul(rho_qn_asym, np.matmul(pl_rot.T, attach_qn_force))
              tension = self.uav_params.mass*cable_len*np.sum(cbl_omg**2) - \
                        np.matmul(xi, qd_u - self.uav_params.mass * self.attach_accel[uav_idx,:])
              tension_vector[:,uav_idx] = tension * xi

              # Sum Net Force, Moment and other corresponding terms
              # for the Payload Dynamics
              pl_net_F = pl_net_F + attach_qn_force
              pl_net_M = pl_net_M + attach_qn_moment
              Ck = self.uav_params.mass * np.matmul(rho_qn_asym, np.matmul(pl_rot.T, xixiT))
              Dk = - np.transpose(Ck)
              Ek = np.matmul(Ck, np.matmul(pl_rot, rho_qn_asym))
              C = C + Ck
              D = D + Dk
              E = E + Ek
              ML = ML + self.uav_params.mass * xixiT
      
      invML = linalg.inv(ML)

      ## Dynamics of Payload
      sdotLoad = self.rigidbody_payloadEOM_readonly(pl_state,pl_net_F,pl_net_M,invML,C,D,E)
      
      self.pl_accel = sdotLoad[3:6]
      self.pl_ang_accel = sdotLoad[10:13]
      
      sdot = sdotLoad
      
      for uav_idx in range(self.nquad):
         uav_s = qd_state[uav_idx,:] 
         if self.cable_is_slack[uav_idx]:
             sdotQuad = self.slack_quadEOM_readonly(uav_s,F,M)
         else:
             T = tension_vector[:,uav_idx]
             F = self.uav_F[uav_idx]
             M = self.uav_M[:,uav_idx]
             sdotQuad = self.taut_quadEOM_readonly(uav_s,F,M,T)
         sdot = np.concatenate((sdot,sdotQuad))

      #print("The sdot for load is", sdotLoad)
      return sdot
      
      #return np.zeros(91)
    
  def rigidbody_payloadEOM_readonly(self, s, F, M, invML, C, D, E): 
      # rigidbody_payloadEOM_READONLY Solve payload equation of motion
      # payloadEOM_readonly calculate the derivative of the state vector
      
      # INPUTS:
      # t      - 1 x 1, time
      # s      - 13 x 1, state vector = [xL, yL, zL, xLd, yLd, zLd, qw, qx, qy, qz, pL,qL,rL]
      # F      - 1 x 1, thrust output from controller (only used in simulation)
      # M      - 3 x 1, moments output from controller (only used in simulation)
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
      omgLdot = np.linalg.solve(effective_inertia,effective_M)

      # Payload Acceleration
      accL = np.matmul(invML, F + np.matmul(D, omgLdot)) - np.array([0,0,self.pl_params.grav])

      # Assemble sdot
      sdotLoad = np.zeros(self.pl_dim_num)
      sdotLoad[0:3] = s[3:6] 
      sdotLoad[3:6] = accL
      sdotLoad[6:10] = qLdot
      sdotLoad[10:13] = omgLdot

      return sdotLoad

  def taut_quadEOM_readonly(self, qd, F, M, T): 
      # QUADEOM_READONLY Solve quadrotor equation of motion when the cable is
      # slack.
      # quadEOM_readonly calculate the derivative of the state vector
          
      # INPUTS:
      # params - struct, parameters you want to pass in
          
          # OUTPUTS:
      # sdot   - 13 x 1, derivative of state vector s
      
      #************ EQUATIONS OF MOTION ************************
      
      # Assign states
      vel   = qd[3:6]
      quat  = qd[6:10]
      omega = qd[10:13]
      R = utilslib.QuatToRot(quat)

      # Acceleration
      #accel = 1 / self.uav_params.mass * (R * np.array([[0],[0],[F]]) + T * xi - np.array([[0],[0],[params.mass * params.grav]]))
      accel =  (np.matmul(R , np.array([0,0,F]) + T)/self.uav_params.mass - np.array([0,0,self.uav_params.grav]))
      
      # quaternion derivative 
      qdot = utilslib.quat_dot(quat,omega)

      # Angular acceleration
      pqrdot = np.matmul(self.uav_params.invI, M - np.cross(omega,np.matmul(self.uav_params.I, omega)))

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
    
  def slack_quadEOM_readonly(self, qd, F, M): 
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
      vel   = qd[3:6]
      quat  = qd[6:10]
      omega = qd[10:13]
      R = utilslib.QuatToRot(quat)

      # Acceleration
      accel =  np.matmul(R , np.array([0,0,F])/self.uav_params.mass - np.array([0,0,self.uav_params.grav]))
      
      # Angular velocity
      qdot = utilslib.quat_dot(quat,omega)

      # Angular acceleration
      pqrdot = np.matmul(self.uav_params.invI , M - np.cross(omega, np.matmul(self.uav_params.I, omega)))

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


####################################################################################
##################                    PTMASS                    ####################
####################################################################################
# partially tested (tested with controller)
  def hybrid_ptmass_pl_transportationEOM(self, t, s):
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
      plqd["qd_rot"] = Rot.T
      quad_load_rel_pos = plqd["qd_pos"]-plqd["pos"]
      quad_load_rel_vel = plqd["qd_vel"]-plqd["vel"]

      if self.cable_is_slack[0]:   
          return self.slack_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F, self.uav_M)
      else:
          plqd["xi"] = -quad_load_rel_pos/l
          plqd["xidot"] = -quad_load_rel_vel/l
          return self.taut_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F, self.uav_M)

  def slack_ptmass_payload_quadEOM_readonly(self, t, plqd, F, M):
      # Assign params and states
      # print("I'm in slack ptmass eom")
      mQ  =   self.uav_params.mass
      e3  =   np.array([[0.0],[0.0],[1.0]])
      g   =   self.uav_params.grav * e3
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
      qdot = -1/2*np.array([[0, -p, -q, -r],
                            [p,  0, -r,  q],
                            [q,  r,  0, -p],
                            [r, -q,  p,  0]]) @ qd_quat.reshape((qd_quat.shape[0], 1)) + K_quat * quaterror * qd_quat.reshape((qd_quat.shape[0], 1))
      '''qdot = -1/2*np.array([[0, -p, -q, -r],
                            [p,  0, -r,  q],
                            [q,  r,  0, -p],
                            [r, -q,  p,  0]]) @ qd_quat + K_quat * quaterror * qd_quat'''

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params.invI @ (M - np.reshape(np.cross(qd_omega, self.uav_params.I @ qd_omega, axisa=0, axisb=0), (3,1)))
      
      # Assemble sdot
      '''print()
      print(plqd["vel"].reshape(3, 1))
      print(plqd["vel"].reshape(3, 1).shape)
      print(g)
      print(g.shape)
      print(plqd["qd_vel"].reshape(3, 1))
      print(plqd["qd_vel"].reshape(3, 1).shape)
      print(accQ)
      print(accQ.shape)
      print(qdot)
      print(qdot.shape)
      print(pqrdot)
      print(pqrdot.shape)
      print()'''
      sdot = np.zeros((26,1), dtype=float)
      sdot[0:3] = plqd["vel"].reshape(3, 1)
      sdot[3:6] = -g
      sdot[13:16] = plqd["qd_vel"].reshape(3, 1)
      sdot[16:19] = accQ
      sdot[19:23] = qdot
      sdot[23:26] = pqrdot
      sdot = sdot[:,0]
      '''
      Moved from taut ptmass
      sdot = np.zeros((26,1), dtype=float)
      sdot[0:3] = plqd["vel"].reshape(3, 1)
      sdot[3:6] = accL
      sdot[13:16] = plqd["qd_vel"].reshape(3, 1)
      sdot[16:19] = accQ
      sdot[19:23] = qdot
      sdot[23:26] = pqrdot
      sdot = sdot[:,0]'''
      '''
      Previously working
      sdot = np.zeros((26,1), dtype=float)
      sdot[0:3, 0] = plqd["vel"]
      sdot[3:6] = -g
      sdot[13:16, 0] = plqd["qd_vel"]
      sdot[16:19, 0:1] = accQ
      sdot[19:23, 0] = qdot
      sdot[23:26] = pqrdot
      sdot = sdot[:,0]'''
      return sdot

  def taut_ptmass_payload_quadEOM_readonly(self, t, plqd, F, M):
      mL = self.pl_params.mass
      mQ = self.uav_params.mass
      total_mass = mL + mQ
      l = self.pl_params.cable_length
      xi = plqd["xi"]
      xidot = plqd["xidot"]
      xi_omega = np.cross(xi,xidot)
      e3=np.array([[0.0],[0.0],[1.0]])
      g = self.uav_params.grav * e3
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
      qdot = -1/2*np.array([[0, -p, -q, -r],
                            [p,  0, -r,  q],
                            [q,  r,  0, -p],
                            [r, -q,  p,  0]]) @ qd_quat.reshape((qd_quat.shape[0], 1)) + K_quat * quaterror * qd_quat.reshape((qd_quat.shape[0], 1))

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params.invI @ (M - np.cross(qd_omega, self.uav_params.I @ qd_omega, axisa=0, axisb=0).T.reshape(3, 1))
      '''   debug print()
      print(plqd["vel"].reshape(3, 1))
      print(plqd["vel"].reshape(3, 1).shape)
      print(accL)
      print(accL.shape)
      print(plqd["qd_vel"].reshape(3, 1))
      print(plqd["qd_vel"].reshape(3, 1).shape)
      print(accQ)
      print(accQ.shape)
      print(qdot)
      print(qdot.shape)
      print(pqrdot)
      print(pqrdot.shape)
      print()'''
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

  def ptmass_inelastic_cable_collision(self, x1, x2, m1, m2): # perfectly inelastic collision of ptmass and the drone along the cable direction
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
# partially tested (tested without controller)
  def rigid_links_cooperative_rigidbody_pl_EOM(self, t, s):
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
      qd = {}
      # extract payload state_vector
      qd["pos"] = s[0:3]
      qd["vel"] = s[3:6]
      qd["quat"] = s[19:23]
      qd["omega"] = s[10:13]
      qd["rot"] = utilslib.QuatToRot(qd["quat"]).T

      result = self.rigidbody_structEOM_readonly(t, qd, self.uav_F, self.uav_M)
      return result
  
  # this is the same as the function in controller.py
  def rigid_links_cooperative_payload_controller_init(self, ql, params):
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
      qLdot = -1/2*np.array([   [0, -p, -q, -r],
                                [p,  0, -r,  q],
                                [q,  r,  0, -p],
                                [r, -q,  p,  0]]) @ quat + K_quat * quaterror * quat

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
  def rpm_command_callback(self,rpm_command,uav_id):
      return

  def fm_command_callback(self,fm_command,uav_id):
        if self.pl_params.mechanism_type == 'Rigid Link':
            self.uav_F[0,0] = fm_command.rlink_thrust.x
            self.uav_F[1,0] = fm_command.rlink_thrust.y
            self.uav_F[2,0] = fm_command.rlink_thrust.z

            self.uav_M[0,0] = fm_command.moments.x
            self.uav_M[1,0] = fm_command.moments.y
            self.uav_M[2,0] = fm_command.moments.z
            #print("self.uav_F", self.uav_F)
            #print("self.uav_M", self.uav_M)
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