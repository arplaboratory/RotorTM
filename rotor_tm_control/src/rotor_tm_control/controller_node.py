#!/usr/bin/python3
import sys
from cv2 import circle
import rospy
import numpy as np
import rospkg
from rotor_tm_control.controller import controller

from nav_msgs.msg import Odometry 
from rotor_tm_msgs.msg import PositionCommand,RPMCommand,FMCommand
from rotor_tm_msgs.msg import CenPL_Command
from geometry_msgs.msg import Vector3

from rotor_tm_utils import read_params
from rotor_tm_utils import utilslib 
from rotor_tm_utils.QuatToRot import QuatToRot
from rotor_tm_utils.vec2asym import vec2asym
from rotor_tm_utils.RotToRPY_ZXY import RotToRPY_ZXY

from rotor_tm_traj.srv import Circle,Line

class controller_node:

    def __init__(self, node_id, single_node = True):
        self.node_id = node_id
        self.single_node = single_node
        self.pl = {}
        self.qd = {}
        self.FM_pub = []
        self.des_odom_pub = []

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rotor_tm_config
        path = rospack.get_path('rotor_tm_config')

        # directory information
        uav_params_path = path + '/config/uav_params/snapdragonfly.yaml'
        payload_params_path = path + '/config/load_params/triangular_payload.yaml'
        mechanism_params_path = path + '/config/attach_mechanism/3_robots_cable_mechanism.yaml'
        payload_control_gain_path = path + '/config/control_params/triangular_payload_cooperative_cable_gains.yaml'
        uav_control_gain_path = path + '/config/control_params/dragonfly_control_gains.yaml'
        self.controller = controller()
        
        # read yaml files
        read_params_funcs = read_params.read_params()
        self.pl_params, self.quad_params = read_params_funcs.system_setup(payload_params_path,uav_params_path,mechanism_params_path, payload_control_gain_path, uav_control_gain_path)

        print("yaml read")
        print("#################")
        print("init contoller_node")
        print()

        if self.single_node:
            ## create a node called 'controller_node'
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
                des_odom_name = mav_name + str(i+1) + "/des_odom"
                self.FM_pub.append(rospy.Publisher(FM_message_name, FMCommand, queue_size=10))
                self.des_odom_pub.append(rospy.Publisher(des_odom_name, Odometry, queue_size=10))

            rospy.spin()
        else:
            print("Using 3 controller nodes")
            ## create a node called 'controller_node'
            node_name = 'controller_'+str(self.node_id+1)
            print(node_name)
            rospy.init_node(node_name)
            node_id += 1
            # TODO: make this to ROS parameters
            mav_name = '/dragonfly'

            # init ROS Subscribers
            rospy.Subscriber('/payload/des_traj', PositionCommand, self.desired_traj_callback)
            rospy.Subscriber('/payload/odom', Odometry, self.pl_odom_callback)

            for uav_id in range(self.pl_params.nquad):
                mav_odom = mav_name + str(uav_id+1) + '/odom'
                self.qd[uav_id] = {}
                rospy.Subscriber(mav_odom, Odometry, self.qd_odom_callback, uav_id)
                
            # init ROS Publishers
            FM_message_name = mav_name + str(self.node_id+1) + "/fm_cmd"
            des_odom_name = mav_name + str(self.node_id+1) + "/des_odom"
            self.cen_pl_cmd_pub = rospy.Publisher(node_name + "/payload/cen_pl_cmd", CenPL_Command, queue_size = 10)
            self.FM_pub.append(rospy.Publisher(node_name +  FM_message_name, FMCommand, queue_size=10))
            # self.des_odom_pub.append(rospy.Publisher(des_odom_name, Odometry, queue_size=10))

            """for i in range(self.pl_params.nquad):
                FM_message_name = mav_name + str(i+1) + "/fm_cmd"
                des_odom_name = mav_name + str(i+1) + "/des_odom"
                self.FM_pub.append(rospy.Publisher(node_name +  FM_message_name, FMCommand, queue_size=10))
                # self.des_odom_pub.append(rospy.Publisher(des_odom_name, Odometry, queue_size=10))"""

            rospy.spin()

    
    def assembly_FM_message(self, F_list, M_list, uav_id):
        FM_message = FMCommand()
        FM_message.thrust = F_list[uav_id]
        FM_message.moments.x = M_list[uav_id][0]
        FM_message.moments.y = M_list[uav_id][1]
        FM_message.moments.z = M_list[uav_id][2]
        return FM_message

    def qd_odom_callback(self, uav_odom, uav_id):
        
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
        #print("qd xi of",uav_id, qd_xi)

        xi = qd_xi.reshape((3,1))
        self.qd[uav_id]["xi"] = xi
        self.qd[uav_id]["xixiT"] = xi @ xi.T
        self.qd[uav_id]["xidot"] = qd_xidot.reshape((3,1))
        self.qd[uav_id]["yaw_des"] = 0
        self.qd[uav_id]["yawdot_des"] = 0

    def pl_odom_callback(self, payload_odom):

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
        self.pl["quat_des"] = np.array([ [1.0],
                                    [0.0],
                                    [0.0],
                                    [0.0]])
        self.pl["omega_des"] = np.array([[0.0, 0.0, 0.0]])
        self.pl["yaw_des"] = 0.0
        self.pl["yawdot_des"] = 0.0
        self.sim_subscriber()

    def controller_setup(self, pl_params):
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
            #print("The uav id is", qn)
            #print("and the qd is", self.qd[qn]["xi"])
        

    def sim_subscriber(self):
        #print("#########################")
        #print("publishing controls...")
        #self.controller_setup(self.pl_params)
        mu, att_acc, F_list, M_list, quat_list, rot_list = self.controller.cooperative_suspended_payload_controller(self.pl, self.qd, self.pl_params, self.quad_params)
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
        #F_list, M_list = self.controller.cooperative_suspended_payload_controller(self.pl, self.qd, self.pl_params, self.quad_params)
        if self.single_node:
            for i in range(self.pl_params.nquad):
                FM_message = self.assembly_FM_message(F_list, M_list, i)
                self.FM_pub[i].publish(FM_message)
                '''des_odom = Odometry()
                des_odom.pose.pose.position.x = rot_list[i][0,0]
                des_odom.pose.pose.position.y = rot_list[i][1,0]
                des_odom.pose.pose.position.z = rot_list[i][2,0]
                des_odom.pose.pose.orientation.x = quat_list[i][0]
                des_odom.pose.pose.orientation.y = quat_list[i][1]
                des_odom.pose.pose.orientation.z = quat_list[i][2]
                des_odom.pose.pose.orientation.w = quat_list[i][3]
                self.des_odom_pub[i].publish(des_odom)'''
        else:
            FM_message = self.assembly_FM_message(F_list, M_list, self.node_id)
            self.FM_pub[0].publish(FM_message)

            #RPM_message_name = '/dragonfly' + str(i+1) + "/rpm_cmd"
            #RPM_pub = rospy.Publisher(RPM_message_name, PositionCommand)
            #RPM_message = assembly_RPM_message(, i)
            #RPM_pub.publish(RPM_message)'''

        '''
        def assembly_RPM_message(self, M_list):
            RPM_message = RPMCommand()

            return RPM_message
        '''


def main(node_id, single_node):
    controller_node(node_id, single_node)


if __name__ == '__main__':
    
    if sys.argv[2] == 1:
        main(int(sys.argv[1]), True)
    else:
        main(int(sys.argv[1]), False)
    
        
