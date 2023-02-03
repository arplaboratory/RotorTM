#!/usr/bin/python3
import sys
from cv2 import circle
import rospy
import numpy as np
import rospkg
from rotor_tm_control.controller import controller

from std_msgs.msg import Bool 
from nav_msgs.msg import Odometry 
from rotor_tm_msgs.msg import PositionCommand,RPMCommand,FMCommand
from rotor_tm_msgs.msg import CenPL_Command
from geometry_msgs.msg import Vector3

from rotor_tm_utils import read_params
from rotor_tm_utils import utilslib 
from rotor_tm_utils.vec2asym import vec2asym

class controller_node:

    def __init__(self, node_id, single_node, payload_params_path, uav_params_path, mechanism_params_path, payload_control_gain_path, uav_control_gain_path):
        self.node_id = node_id
        self.single_node = single_node
        self.pl = {}
        self.qd = {}
        self.FM_pub = []
        self.last_odom_time_received = 0.0 
        self.last_des_traj_time_received = 0.0 
        self.controller = controller()
        
        # read yaml files
        read_params_funcs = read_params.read_params()
        self.pl_params, self.quad_params = read_params_funcs.system_setup(payload_params_path,uav_params_path,mechanism_params_path, payload_control_gain_path, uav_control_gain_path)

        print("yaml read")
        print("#################")
        print("init contoller_node")
        print()

        # the controller node was designed to be launched once or multiple times
        # if only launched once, (all uavs share the same controller node) 
        # enters the first if branch
        # if launched more than once, (each uav uses its own controller node)
        # enters the second else branch
        if self.single_node:
            # Currently controllers are launch multiple times, (each uav uses its own controller)
            # the section below is not in use
            ##
            ## Important Notice:
            ## to make topic name consistant with multi_node case
            ## so that simulation_base does not need to differeinate single_node and multi_node case
            ## the published topics from this single node has the same naming convention
            ## as the multi_node case:
            ## controller_#/dragonfly#/fm_cmd
            ## please note that all of these are topics under a single node
            ##
            
            rospy.init_node('controller_node')
            # TODO: make this to ROS parameters
            mav_name = 'dragonfly'
            
            # init ROS Subscribers
            rospy.Subscriber('/payload/des_traj', PositionCommand, self.desired_traj_callback)
            rospy.Subscriber('/payload/odom', Odometry, self.pl_odom_callback)

            for uav_id in range(self.pl_params.nquad):
                mav_odom = mav_name + str(uav_id+1) + '/odom'
                self.qd[uav_id] = {}
                rospy.Subscriber(mav_odom, Odometry, self.qd_odom_callback, uav_id)
            
            # init ROS Publishers
            self.cen_pl_cmd_pub = rospy.Publisher("/payload/cen_pl_cmd", CenPL_Command, queue_size = 10)
            for i in range(self.pl_params.nquad):
                FM_message_name = mav_name + str(i+1) + "/fm_cmd"
                FM_prefix = "controller_"+str(i+1)+"/"
                self.FM_pub.append(rospy.Publisher(FM_prefix + FM_message_name, FMCommand, queue_size=10))

            rospy.spin()
        else:
            # Currently controllers are launch multiple times, (each uav uses its own controller)
            # the section below is in use
            ##
            ## the published topics from each launched node has the same naming convention
            ## as the single_node case:
            ## controller_#/dragonfly#/fm_cmd
            ## However, controller_x publishes only controller_x/dragonflyx/fm_cmd message
            ## e.g.
            ## node: controller_1
            ## Only Publication: controller_1/dragonfly1/fm_cmd message
            ##

            print("Using multiple controller nodes")

            # init ROS Subscribers
            rospy.Subscriber('/payload/des_traj', PositionCommand, self.desired_traj_callback, queue_size=1, tcp_nodelay=True)
            rospy.Subscriber('/payload/odom', Odometry, self.pl_odom_callback, queue_size=1,tcp_nodelay=True)

            uav_name = self.pl_params.uav_in_team[node_id]
            uav_odom = '/' + uav_name + '/odom'
            self.qd[node_id] = {}
            rospy.Subscriber(uav_odom, Odometry, self.qd_odom_callback, node_id, queue_size=1,tcp_nodelay=True)
                
            # init ROS Publishers
            FM_message_name = '/' + uav_name + "/fm_cmd"
            des_odom_name = '/' + uav_name + "/des_odom"
            self.cen_pl_cmd_pub = rospy.Publisher(node_name + "/payload/cen_pl_cmd", CenPL_Command, queue_size = 10)
            self.FM_pub.append(rospy.Publisher(node_name +  FM_message_name, FMCommand, queue_size=1, tcp_nodelay=True))
            self.status_pub = rospy.Publisher(node_name + "/heartbeat", Bool, queue_size = 10)

            rate = rospy.Rate(100)
            while not rospy.is_shutdown():
                # Publish if MPC is busy with a current trajectory
                msg = Bool()
                msg.data = True 
                self.status_pub.publish(msg)
                rate.sleep()
    
    def assembly_FM_message(self, F_list, M_list, uav_id):
    # DESCRIPTION:
    # Assembly function for sending out FM message
    # The function automatically detects the controller used
    # and assembly their respective FM_message from controller output
    # There are three scenarios: (1) Point mass, (2) cooperative cable suspended, and (3) cooperative rigidlink connect

    # INPUTS:
    # F_list        - Thrust/Force output from controller, type and format vary based on scenarios, check controller OUTPUTS for detail
    # M_list        - Moment output from controller, type and format vary based on scenarios, check controller OUTPUTS for detail
    # uav_id        - an integer identifying the id of the current MAV the controller is controlling          

    # OUTPUTS:
    # FM_message    - a FMCommand type object, accepted by the simulator
    #                 refer to ~workspace/src/RotorTM/rotor_tm_msgs/msg/FMCommand.msg for detail
        if self.pl_params.mechanism_type == 'Rigid Link':
            FM_message = FMCommand()
            FM_message.rlink_thrust.x = F_list[0][0]
            FM_message.rlink_thrust.y = F_list[0][1]
            FM_message.rlink_thrust.z = F_list[0][2]
            FM_message.moments.x = M_list[0][0]
            FM_message.moments.y = M_list[0][1] 
            FM_message.moments.z = M_list[0][2]
            return FM_message
        elif self.pl_params.mechanism_type == 'Cable':
            FM_message = FMCommand()
            FM_message.header.stamp = rospy.get_rostime()
            if self.pl_params.payload_type == 'Rigid Body':
                FM_message.thrust = F_list[uav_id]
                FM_message.moments.x = M_list[uav_id][0]
                FM_message.moments.y = M_list[uav_id][1]
                FM_message.moments.z = M_list[uav_id][2]
                return FM_message
            elif self.pl_params.payload_type == 'Point Mass':
                FM_message.thrust = F_list[0,0]
                FM_message.moments.x = M_list[0,0][0]
                FM_message.moments.y = M_list[1,0][0]
                FM_message.moments.z = M_list[2,0][0]
                return FM_message

    def qd_odom_callback(self, uav_odom, uav_id):
    # DESCRIPTION:
    # Callback function for rospy subscriber to extract MAV odometry

    # INPUTS:
    # uav_odom      - nav_msgs.msg Odometry type message published by simulation Node (/sim)
    # uav_id        - an integer used to identify the MAV subscribing

    # OUTPUTS:
    # self.qd       - update self.qd with current Odometry of the MAV

        self.qd[uav_id]["pos"] = np.array( [[uav_odom.pose.pose.position.x],
                                         [uav_odom.pose.pose.position.y],
                                         [uav_odom.pose.pose.position.z]])
        self.qd[uav_id]["vel"] = np.array( [[uav_odom.twist.twist.linear.x],
                                         [uav_odom.twist.twist.linear.y],
                                         [uav_odom.twist.twist.linear.z]]) 
        self.qd[uav_id]["quat"] = np.array([[uav_odom.pose.pose.orientation.w],
                                         [uav_odom.pose.pose.orientation.x],
                                         [uav_odom.pose.pose.orientation.y],
                                         [uav_odom.pose.pose.orientation.z]]) 
        self.qd[uav_id]["omega"] = np.array([[uav_odom.twist.twist.angular.x],
                                            [uav_odom.twist.twist.angular.y],
                                            [uav_odom.twist.twist.angular.z]]) 
        Rot = utilslib.QuatToRot(self.qd[uav_id]["quat"])
        self.qd[uav_id]["rot"] = Rot
        rho_vec = self.pl_params.rho_vec_list[:,uav_id].reshape((3,1))
        cable_len = self.pl_params.cable_length[uav_id]
        pl_rot = self.pl["rot"]
        pl_pos = self.pl["pos"]
        pl_omega_asym = vec2asym(self.pl["omega"])

        robot_attach_vector = self.pl["pos"] + pl_rot @ rho_vec - self.qd[uav_id]["pos"]
        qd_xi = robot_attach_vector / np.linalg.norm(robot_attach_vector) 
        qd_xidot = (self.pl["vel"] + pl_rot @ pl_omega_asym @ rho_vec - self.qd[uav_id]["vel"]) #/ cable_len

        xi = qd_xi.reshape((3,1))
        self.qd[uav_id]["xi"] = xi
        self.qd[uav_id]["xixiT"] = xi @ xi.T
        self.qd[uav_id]["xidot"] = qd_xidot.reshape((3,1))
        self.qd[uav_id]["yaw_des"] = 0
        self.qd[uav_id]["yawdot_des"] = 0

    def pl_odom_callback(self, payload_odom):
    # DESCRIPTION:
    # Callback function for rospy subscriber to extract payload odometry

    # INPUTS:
    # payload_odom  - nav_msgs.msg Odometry type message published by simulation Node (/sim)

    # OUTPUTS:
    # self.pl       - update self.pl with current Odometry of the payload
        self.pl["pos"] = np.array([     [payload_odom.pose.pose.position.x],
                                        [payload_odom.pose.pose.position.y],
                                        [payload_odom.pose.pose.position.z]])

        self.pl["vel"] = np.array([     [payload_odom.twist.twist.linear.x],
                                        [payload_odom.twist.twist.linear.y],
                                        [payload_odom.twist.twist.linear.z]])
                                
        self.pl["quat"] = np.array([    [payload_odom.pose.pose.orientation.w],
                                        [payload_odom.pose.pose.orientation.x],
                                        [payload_odom.pose.pose.orientation.y],
                                        [payload_odom.pose.pose.orientation.z]])
                                
        self.pl["omega"] = np.array([   [payload_odom.twist.twist.angular.x],
                                        [payload_odom.twist.twist.angular.y],
                                        [payload_odom.twist.twist.angular.z]])
        self.pl["rot"] = utilslib.QuatToRot(self.pl["quat"])

    def desired_traj_callback(self, des_traj):
    # DESCRIPTION:
    # Callback function for rospy subscriber to extract desired trajectory

    # INPUTS:
    # des_traj      - rotor_tm_msgs.msg PositionCommand type message published by trajectory Node (/des_traj)

    # OUTPUTS:
    # self.pl       - update self.pl with the desired trajectory of the payload
        self.pl["pos_des"] = np.array([ [des_traj.position.x],
                                        [des_traj.position.y],
                                        [des_traj.position.z]])
        self.pl["vel_des"] = np.array([ [des_traj.velocity.x],
                                        [des_traj.velocity.y],
                                        [des_traj.velocity.z]])
        self.pl["acc_des"] = np.array([ [des_traj.acceleration.x],
                                        [des_traj.acceleration.y],
                                        [des_traj.acceleration.z]])     
        self.pl["jrk_des"] = np.array([ [des_traj.jerk.x],
                                        [des_traj.jerk.y],
                                        [des_traj.jerk.z]])                     
        self.pl["quat_des"] = np.array([[des_traj.quaternion.w],
                                        [des_traj.quaternion.x],
                                        [des_traj.quaternion.y],
                                        [des_traj.quaternion.z]])
        self.pl["omega_des"] = np.array([[des_traj.angular_velocity.x],
                                         [des_traj.angular_velocity.y],
                                         [des_traj.angular_velocity.z]])
        self.pl["yaw_des"] = 0.0
        self.pl["yawdot_des"] = 0.0
        self.sim_subscriber()
    
    def controller_setup(self, pl_params):
    # DESCRIPTION:
    # Prepares input for cooperative cable suspended payload scenario controller
       
    # INPUTS:
    # pl_params             - a read_params class object containing payload parameters        

    # OUTPUTS:
    # self.qd(modified)     - self.qd is modified to meet the specification of the INPUT 
    #                         for cooperative_suspended_payload_controller. Specifically,
    #                         self.qd is a list of dictionary containing states of all MAV(s)
    #                         self.qd[0] would give a dictionary of MAV 0's states and related 
    #                         information. Specifically,
    #                         Key             Type            Size            Description              
    #                         'pos'           ndarray         3 by 1          MAV 0's position
    #                         'vel'           ndarray         3 by 1          MAV 0's velocity
    #                         'quat'          ndarray         4 by 1          MAV 0's orientation as unit quaternion
    #                         'omega'         ndarray         3 by 1          MAV 0's angular velocity
    #                         'rot'           ndarray         3 by 3          MAV 0's rotation as rotation matrix
    #                         'xi'            ndarray         3 by 1          MAV 0's cable direction as a unit vector
    #                         'xixiT'         ndarray         3 by 3          xi dot product with xi
    #                         'xidot'         ndarray         3 by 1          MAV 0's velocity normalized over separation distance
    #                         'yaw_des'       float           NA              desired payload yaw, set to 0.0 current
    #                         'yawdot_des'    float           NA              time derivative of desired payload yaw, set to 0.0 currently
    #                         'mu_des'        ndarray         3 by 1          desired cable tension of the cable suspended under MAV 0
    #                         'attach_accel'  ndarray         3 by 1          acceleration of the cable attach point
    #                         'rot_des'       ndarray         3 by 3          desired rotation as a rotation matrix
    #                         'omega_des'     ndarray         3 by 1          desired payload angular velocity
    #                                                                         set to [[0., 0., 0.]] currently
        rho_vec_list = pl_params.rho_vec_list
        cable_len_list = pl_params.cable_length
        pl_rot = self.pl["rot"]
        pl_omega_asym = vec2asym(self.pl["omega"])

        qd_pos = np.zeros((pl_params.nquad, 0), dtype=float)
        for i in range(pl_params.nquad):
            qd_pos = np.append(qd_pos, self.qd[i]["pos"], axis = 1)
        
        qd_vel = np.zeros((pl_params.nquad, 0), dtype=float)
        for i in range(pl_params.nquad):
            qd_vel = np.append(qd_vel, self.qd[i]["vel"], axis = 1)

        robot_attach_vector = self.pl["pos"] + pl_rot @ rho_vec_list - qd_pos
        temp = np.array([[]])
        for i in range(robot_attach_vector.shape[1]):
            temp = np.append(np.array([[np.linalg.norm(robot_attach_vector[:,i])]]), temp, axis=1)
        qd_xi = robot_attach_vector / temp

        qd_xidot = (self.pl["vel"] + pl_rot @ pl_omega_asym @ rho_vec_list - qd_vel) / cable_len_list

        for qn in range(pl_params.nquad):
            xi = qd_xi[:, qn].reshape((3,1))
            self.qd[qn]["xi"] = xi
            self.qd[qn]["xixiT"] = xi @ xi.T
            self.qd[qn]["xidot"] = qd_xidot[:, qn].reshape((3,1))
            self.qd[qn]["yaw_des"] = 0
            self.qd[qn]["yawdot_des"] = 0

    def assembly_plqd(self):
    # DESCRIPTION:
    # Prepares input for point mass scenario controller
    
    # INPUTS:
    # /                     - Extract self.pl to assemble the required input for point mass scenario controller      

    # OUTPUTS:
    # plqd                  - plqd is created from self.pl to meet the specification of the INPUT 
    #                         for single_payload_geometric_controller. Specifically,
    #                         plqd is a dictionary containing state of the payload and MAV combined, specifically
    #                         Key             Type            Size            Description              
    #                         'pos'           ndarray         3 by 1          payload position
    #                         'vel'           ndarray         3 by 1          payload velocity
    #                         'qd_pos'        ndarray         3 by 1          MAV position
    #                         'qd_vel'        ndarray         3 by 1          MAV velocity
    #                         'qd_quat'       ndarray         4 by 1          MAV orientation as unit quaternion
    #                         'qd_omega'      ndarray         3 by 1          MAV angular velocity
    #                         'qd_rot'        ndarray         3 by 3          MAV orientation as rotation matrix
    #                         'pos_des'       ndarray         3 by 1          desired payload position
    #                         'vel_des'       ndarray         3 by 1          desired payload velocity
    #                         'acc_des'       ndarray         3 by 1          desired payload acceleration
    #                         'jrk_des'       ndarray         3 by 1          desired payload jerk
    #                         'quat_des'      ndarray         4 by 1          desired payload orientation as unit quaterion
    #                                                                         set to [[1.], [0.], [0.], [0.]] currently
    #                         'omega_des'     ndarray         3 by 1          desired payload angular velocity
    #                                                                         set to [[0., 0., 0.]] currently
    #                         'qd_yaw_des'    float           NA              desired MAV yaw, set to 0.0 current
    #                         'qd_yawdot_des' float           NA              time derivative of desired MAV yaw, set to 0.0 currently
        plqd = {}
        plqd["pos"] = self.pl["pos"]
        plqd["vel"] = self.pl["vel"]
        plqd["qd_pos"] = self.qd[0]["pos"]
        plqd["qd_vel"] = self.qd[0]["vel"]
        plqd["qd_quat"] = self.qd[0]["quat"]
        plqd["qd_omega"] = self.qd[0]["omega"]
        plqd["qd_rot"] = self.qd[0]["rot"]
        plqd["pos_des"] = self.pl["pos_des"]
        plqd["vel_des"] = self.pl["vel_des"]
        plqd["acc_des"] = self.pl["acc_des"]
        plqd["jrk_des"] = self.pl["jrk_des"]
        plqd["qd_yaw_des"] = self.qd[0]["yaw_des"] 
        plqd["qd_yawdot_des"] = self.qd[0]["yawdot_des"]
        plqd["quat_des"] = self.pl["quat_des"]
        plqd["omega_des"] = self.pl["omega_des"]
        return plqd

    def assembly_qd(self):
    # DESCRIPTION:
    # Prepares input for cooperative rigid link connected payload scenario controller
    
    # INPUTS:
    # /                     - Extract self.pl to assemble the required input for rigid link scenario controller      

    # OUTPUTS:
    # qd_state              - qd_state is created from self.pl to meet the specification of the INPUT 
    #                         for rigid_links_cooperative_payload_controller. Specifically,
    #                         qd_state is a dictionary containing state of the payload, specifically
    #                         Key             Type            Size            Description              
    #                         'pos'           ndarray         3 by 1          payload position
    #                         'vel'           ndarray         3 by 1          payload velocity
    #                         'quat'          ndarray         4 by 1          payload orientation as unit quaternion
    #                         'omega'         ndarray         3 by 1          payload angular velocity
    #                         'rot'           ndarray         3 by 3          payload rotation as rotation matrix
    #                         'pos_des'       ndarray         3 by 1          desired payload position
    #                         'vel_des'       ndarray         3 by 1          desired payload velocity
    #                         'acc_des'       ndarray         3 by 1          desired payload acceleration
    #                         'jrk_des'       ndarray         3 by 1          desired payload jerk
    #                         'quat_des'      ndarray         4 by 1          desired payload orientation as unit quaterion
    #                                                                         set to [[1.], [0.], [0.], [0.]] currently
    #                         'omega_des'     ndarray         3 by 1          desired payload angular velocity
    #                                                                         set to [[0., 0., 0.]] currently
    #                         'qd_yaw_des'    float           NA              desired MAV yaw, set to 0.0 current
    #                         'qd_yawdot_des' float           NA              time derivative of desired MAV yaw, set to 0.0 currently
        qd_state = {}
        # init uav_F and uav_M
        qd_state["pos"] = self.pl["pos"]
        qd_state["vel"] = self.pl["vel"]
        qd_state["quat"] = self.pl["quat"]
        qd_state["omega"] = self.pl["omega"]
        qd_state["rot"] = utilslib.QuatToRot(qd_state["quat"])

        qd_state["pos_des"] = self.pl["pos_des"]
        qd_state["vel_des"] = self.pl["vel_des"]
        qd_state["acc_des"] = self.pl["acc_des"] 
        qd_state["jrk_des"] = self.pl["jrk_des"]
        qd_state["qd_yaw_des"] = 0.0
        qd_state["qd_yawdot_des"] = 0.0
        qd_state["quat_des"] = self.pl["quat_des"]
        qd_state["omega_des"] = self.pl["omega_des"]
        return qd_state

    # this function take odometry from simualtion, call respective controller, and publish FM_cmd
    def sim_subscriber(self):
    # DESCRIPTION:
    # This method takes odometry from simualtion, assemble 
    # input for the respective controllers using the above methods
    # and call respective controller. The return values from the controller
    # are then packaged into a FMCommand type ros message and published under /controller_#
    
    # INPUTS:
    # /                     - Attributes of the controller class are used      

    # OUTPUTS:
    # /                     - FMCommands are published
        if self.pl_params.mechanism_type == 'Rigid Link':
            ql = self.assembly_qd()
            F_list, M_list = self.controller.rigid_links_cooperative_payload_controller(ql, self.pl_params)
        elif self.pl_params.mechanism_type == 'Cable':
            if self.pl_params.payload_type == 'Rigid Body':
                mu, att_acc, F_list, M_list, quat_list, rot_list = self.controller.cooperative_suspended_payload_controller(self.pl, self.qd, self.pl_params, self.quad_params, self.node_id)
                cen_pl_command = CenPL_Command()
                cen_pl_command.header.stamp = rospy.get_rostime()
                cen_pl_command.header.frame_id = "simulator" 
                cen_pl_command.copr_status = 3
                for i in range(self.pl_params.nquad):
                    acc_command = Vector3()
                    acc_command.x = att_acc[0,i]
                    acc_command.y = att_acc[1,i]
                    acc_command.z = att_acc[2,i]

                    mu_command = Vector3()
                    mu_command.x = mu[3*i,0]
                    mu_command.y = mu[3*i+1,0]
                    mu_command.z = mu[3*i+2,0]

                    cen_pl_command.acc.append(acc_command)
                    cen_pl_command.mu.append(mu_command)
                    cen_pl_command.estimated_acc.append(acc_command)

                self.cen_pl_cmd_pub.publish(cen_pl_command)
            elif self.pl_params.payload_type == 'Point Mass':
                plqd = self.assembly_plqd()
                F_list, M_list = self.controller.single_payload_geometric_controller(ql = plqd, pl_params = self.pl_params, qd_params = self.quad_params)
        
        if self.single_node:
            print("The self.single_node is ", self.single_node)
            for i in range(self.pl_params.nquad):
                FM_message = self.assembly_FM_message(F_list, M_list, i)
                self.FM_pub[i].publish(FM_message)
        else:
            FM_message = self.assembly_FM_message(F_list, M_list, self.node_id)
            self.FM_pub[0].publish(FM_message)

if __name__ == '__main__':
    # This section is used to configure the launch file
    payload_params_path = sys.argv[3]
    uav_params_path = sys.argv[4]
    mechanism_params_path = sys.argv[5]
    payload_control_gain_path = sys.argv[6]
    uav_control_gain_path = sys.argv[7]

    node_name = 'controller_'+str(int(sys.argv[1])+1)
    print(node_name)
    rospy.init_node(node_name)

    if int(sys.argv[2]) == 1:
        controller_node(int(sys.argv[1]), True, payload_params_path, uav_params_path, mechanism_params_path, payload_control_gain_path, uav_control_gain_path)
    else:
        controller_node(int(sys.argv[1]), False, payload_params_path, uav_params_path, mechanism_params_path, payload_control_gain_path, uav_control_gain_path)
