import numpy as np
import numpy.linalg as linalg
from yaml.error import Mark
import rospy
import scipy.integrate
from scipy.spatial.transform import Rotation as rot_math
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry 
from rotor_tm_msgs.msg import RPMCommand, FMCommand 
from rotor_tm_utils import utilslib, rosutilslib, QuatToRot
import time

class simulation_base():
  def __init__(self,pl_params,uav_params):
      rospy.init_node('simulation')
      self.rate = 2000
      rate = rospy.Rate(self.rate) # 1000hz
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

      if self.nquad != 1:
        self.uav_F = np.matmul(self.pl_params.pseudo_inv_P, np.array([0,0,self.pl_params.mass * self.pl_params.grav,0,0,0])) + np.kron([1]*self.nquad, [0,0,self.uav_params.mass * self.pl_params.grav]) 
        self.uav_F = self.uav_F.reshape(3,self.nquad)[:,2]
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
        '''
        Need a custimized init for ptmass case. Publishing data needs to be changed as well.
        Old state is (13, 1)
        New state is (19, 1)
        self.uav_F = np.zeros((3, 1), dtype=float)
        self.uav_M = np.zeros((3,self.nquad), dtype=float)
        self.uav_M = np.zeros((3,self.nquad))
        self.cable_is_slack = np.zeros(self.nquad)

        self.rho_vec_list = self.pl_params.rho_vec_list
        self.cable_len_list = np.array(self.pl_params.cable_length)

        x = np.array([0,0,0,0,0,0,0,0,0.5,0,0,0,1.0,0,0,0,0,0,0])'''

      # ROS Publisher 
      self.system_publisher = rospy.Publisher('system/marker',MarkerArray,queue_size=10)
      self.payload_odom_publisher = rospy.Publisher('payload/odom',Odometry,queue_size=10)
      self.robot_odom_publisher = []
      self.attach_publisher = []
      for i in range(self.nquad):
          self.robot_odom_publisher.append(rospy.Publisher(self.mav_name + str(i+1) + '/odom',Odometry,queue_size=10))
          self.attach_publisher.append(rospy.Publisher(self.mav_name + str(i+1) + '/attach',Odometry,queue_size=10))
           

      # ROS Subscriber 
      self.robot_command_subscriber = []
      for uav_id in range(self.nquad):
          mav_name = self.mav_name + str(uav_id+1)
          self.robot_command_subscriber.append(rospy.Subscriber('/'+mav_name + '/rpm_cmd',RPMCommand,self.rpm_command_callback,uav_id))
          self.robot_command_subscriber.append(rospy.Subscriber('/'+mav_name + '/fm_cmd',FMCommand,self.fm_command_callback,uav_id))
    
      # Visualization Init
      self.cable_marker_scale = 0.01 * np.ones(3)
      self.cable_marker_color = np.array([1.0,0.5,0.5,0.5])
      self.uav_marker_scale = 0.5 * np.ones(3)
      self.uav_marker_color = np.array([1.0,0.0,0.0,1.0])
      self.uav_mesh = self.uav_params.mesh_path
      self.payload_marker_scale = np.ones(3)
      self.payload_marker_color = np.array([1.0,0.745,0.812,0.941])
      self.payload_mesh = self.pl_params.mesh_path
      self.payload_marker_msg = rosutilslib.init_marker_msg(Marker(),10,0,self.worldframe,self.payload_marker_scale,self.payload_marker_color,self.payload_mesh)

      while not rospy.is_shutdown():
        # Without event
        #tsave, xsave = scipy.integrate.solve_ivp(rotortm_simulation_base.run(), t_span, x, method='RK45', event = simulation_base.guard())
        # With event
        start = time.time()
        if self.nquad == 1:
            sol = scipy.integrate.solve_ivp(self.hybrid_ptmass_pl_transportationEOM, t_span, x, method= 'RK45', t_eval=t_span)
        else:    
            sol = scipy.integrate.solve_ivp(self.hybrid_cooperative_rigidbody_pl_transportationEOM, t_span, x, method='RK45', t_eval=t_span)
        end = time.time()
        x = sol.y[:,1]

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

        payload_rotmat = utilslib.QuatToRot(sol.y[:,0][6:10])

        system_marker = MarkerArray()
        cable_point_list = np.zeros((2*self.nquad,3))
        for uav_id in range(self.nquad):
            # Publish UAV odometry
            uav_odom = Odometry()
            uav_odom.header.stamp = current_time
            uav_odom.header.frame_id = self.worldframe 
            uav_state = x[self.pl_dim_num+self.uav_dim_num*uav_id:self.pl_dim_num+self.uav_dim_num*(uav_id+1)]
            #print("The uav", uav_id + 1, "_state is",uav_state)
            #print("The full state is",x)
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

            # Publish UAV odometry
            attach_odom = Odometry()
            attach_odom.header.stamp = current_time
            attach_odom.header.frame_id = self.worldframe 
            #TODO: continue here publish the attach odom. 
            attach_pos = x[0:3] + np.matmul(payload_rotmat, self.rho_vec_list[:,uav_id])
            attach_vel = x[3:6] + np.matmul(payload_rotmat, np.cross(sol.y[:,0][10:13], self.rho_vec_list[:,uav_id]))
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

  def hybrid_ptmass_pl_transportationEOM(self, t, s):
      l = self.pl_params.cable_length
      plqd = {}
      # convert state s to plqd
      plqd["pos"] = s[0:3]
      plqd["vel"] = s[3:6]
      plqd["qd_pos"] = s[6:9]
      plqd["qd_vel"] = s[9:12]
      plqd["qd_quat"] = s[12:16]
      plqd["qd_omega"] = s[16:19]
      Rot = utilslib.QuatToRot(s[12:16])
      plqd["qd_rot"] = Rot.T
      quad_load_rel_pos = plqd["qd_pos"]-plqd["pos"]
      quad_load_rel_vel = plqd["qd_vel"]-plqd["vel"]

      if not self.cable_is_slack[0]:          
          return self.slack_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F[0,0], self.uav_M)
      else:
          plqd["xi"] = -quad_load_rel_pos/l
          plqd["xidot"] = -quad_load_rel_vel/l
          return self.taut_ptmass_payload_quadEOM_readonly(t, plqd, self.uav_F[0,0], self.uav_M)

  def slack_ptmass_payload_quadEOM_readonly(self, t, plqd, F, M):
      # Assign params and states
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
                            [r, -q,  p,  0]]) @ qd_quat + K_quat * quaterror * qd_quat

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params.invI @ (M - np.reshape(np.cross(qd_omega, self.uav_params.I @ qd_omega, axisa=0, axisb=0), (3,1)))
      
      # Assemble sdot
      sdot = np.zeros((19,1), dtype=float)
      sdot[0:3, 0] = plqd["vel"]
      sdot[3:6] = -g
      sdot[6:9, 0] = plqd["qd_vel"]
      sdot[9:12, 0:1] = accQ
      sdot[12:16, 0] = qdot
      sdot[16:19] = pqrdot
      sdot = sdot[:,0]

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
      g = self.uav_params.grav @ e3
      wRb=plqd["qd_rot"]    #Rotation matrix of the quadrotor
      qd_quat = plqd["qd_quat"]
      qd_omega = plqd["qd_omega"]
      p = qd_omega[0]
      q = qd_omega[1]
      r = qd_omega[2]

      # Obtain tension vector
      quad_force_vector = F * wRb @ e3
      quad_centrifugal_f = mQ @ l @ (xi_omega.T @ xi_omega)
      tension_vector = mL * (-xi.T @ quad_force_vector + quad_centrifugal_f) @ xi / total_mass

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
                            [r, -q,  p,  0]]) @ qd_quat + K_quat @ quaterror @ qd_quat

      # Solving for Quadrotor Angular Acceleration
      pqrdot   = self.uav_params.invI * (M - np.cross(qd_omega, self.uav_params.I @ qd_omega))

      # Assemble sdot
      sdot = np.zeros((19,1), dtype=float)
      sdot[0:3] = plqd["vel"]
      sdot[3:6] = accL
      sdot[6:9] = plqd["qd_vel"]
      sdot[9:12] = accQ
      sdot[12:16] = qdot
      sdot[16:19] = pqrdot

      return sdot

  def rpm_command_callback(self,rpm_command,uav_id):
      return

  def fm_command_callback(self,fm_command,uav_id):
      # print("I am getting the thrust and moment command for UAV")
      self.uav_F[uav_id] = fm_command.thrust
      self.uav_M[0,uav_id] = fm_command.moments.x
      self.uav_M[1,uav_id] = fm_command.moments.y
      self.uav_M[2,uav_id] = fm_command.moments.z

