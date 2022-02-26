#!/usr/bin/python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry 
from rotor_tm_utils import read_params
from rotor_tm_control import controller
from rotor_tm_msgs.msg import PositionCommand
from rotor_tm_msgs.msg import RPMCommand
from rotor_tm_msgs.msg import FMCommand
from rotor_tm_utils.QuatToRot import QuatToRot
from rotor_tm_utils.vec2asym import vec2asym
from rotor_tm_utils.RotToRPY_ZXY import RotToRPY_ZXY


class controller_node:

    def __init__(self):

        self.pl = {}
        self.qd = {}
        self.FM_pub = []

        # directory information
        uav_params_path = '/home/thomas_ubuntu/catkin_ws/src/RotorTM/config/uav_params/snapdragonfly.yaml'
        payload_params_path = '/home/thomas_ubuntu/catkin_ws/src/RotorTM/config/load_params/triangular_payload.yaml'
        mechanism_params_path = '/home/thomas_ubuntu/catkin_ws/src/RotorTM/config/attach_mechanism/3_robots_cable_mechanism.yaml'
        payload_control_gain_path = '/home/thomas_ubuntu/catkin_ws/src/RotorTM/config/control_params/triangular_payload_cooperative_cable_gains.yaml'
        uav_control_gain_path = '/home/thomas_ubuntu/catkin_ws/src/RotorTM/config/control_params/dragonfly_control_gains.yaml'
        
        # read yaml files
        read_params_funcs = read_params.read_params()
        self.pl_params, self.quad_params = read_params_funcs.system_setup(payload_params_path,uav_params_path,mechanism_params_path, payload_control_gain_path, uav_control_gain_path)

        print("yaml read")
        print("#################")
        print("init contoller_node")
        print()
        ## create a node called 'controller_node'
        rospy.init_node('controller_node')
        print("#################")
        print("listening for messages...")
        print()
        # init subscribers
        rospy.Subscriber('/payload/des_traj', PositionCommand, self.desired_traj_callback)
        rospy.Subscriber('/payload/odom', Odometry, self.pl_odom_callback)

        for uav_id in range(self.pl_params.nquad):
            mav_name = 'dragonfly' + str(uav_id+1) + '/odom'
            rospy.Subscriber(mav_name, Odometry, self.qd_odom_callback, (uav_id, self.pl_params.nquad))
        
        # init publishers
        for i in range(self.pl_params.nquad):
            FM_message_name = '/dragonfly' + str(i+1) + "/fm_cmd"
            self.FM_pub.append(rospy.Publisher(FM_message_name, FMCommand, queue_size=10))

        rospy.spin()

    def assembly_FM_message(self, F_list, M_list, uav_id):
        FM_message = FMCommand()
        FM_message.thrust = F_list[uav_id]
        FM_message.moments.x = M_list[uav_id][0]
        FM_message.moments.y = M_list[uav_id][1]
        FM_message.moments.z = M_list[uav_id][2]
        return FM_message
    
    def qd_odom_callback(self, uav_odom, curr_and_max):
        arg = curr_and_max[0]
        max = curr_and_max[1]
        self.qd[arg] = {}
        self.qd[arg]["pos"] = np.array( [[uav_odom.pose.pose.position.x],
                                         [uav_odom.pose.pose.position.y],
                                         [uav_odom.pose.pose.position.z]])
        self.qd[arg]["vel"] = np.array( [[uav_odom.twist.twist.linear.x],
                                         [uav_odom.twist.twist.linear.y],
                                         [uav_odom.twist.twist.linear.z]]) 
        self.qd[arg]["quat"] = np.array([[uav_odom.pose.pose.orientation.w],
                                         [uav_odom.pose.pose.orientation.x],
                                         [uav_odom.pose.pose.orientation.y],
                                         [uav_odom.pose.pose.orientation.z]]) 
        self.qd[arg]["omega"] = np.array( [ [uav_odom.twist.twist.angular.x],
                                            [uav_odom.twist.twist.angular.y],
                                            [uav_odom.twist.twist.angular.z]]) 
        Rot = QuatToRot(self.qd[arg]["quat"])
        phi, theta, yaw = RotToRPY_ZXY(Rot)
        self.qd[arg]["euler"] = np.array([[phi],[theta],[yaw]])
        self.qd[arg]["rot"] = Rot.T

        #if arg == max-1:
         #   self.sim_subscriber()

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
        self.pl["rot"] = QuatToRot(self.pl["quat"])
        self.pl["quat_des"] = np.array([ [1.0],
                                    [0.0],
                                    [0.0],
                                    [0.0]])
        self.pl["omega_des"] = np.array([[0.0, 0.0, 0.0]])

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

        for qn in range(1, pl_params.nquad+1):
            self.qd[qn-1]["xi"] = qd_xi[:, qn-1:qn]
            self.qd[qn-1]["xixiT"] = qd_xi[:, qn-1:qn] @ qd_xi[:, qn-1:qn].T
            self.qd[qn-1]["xidot"] = qd_xidot[:, qn-1:qn]
            self.qd[qn-1]["yaw_des"] = 0
            self.qd[qn-1]["yawdot_des"] = 0

    '''    print()
        print(pl["vel"])
        print()
        print(pl_rot)
        print()
        print(pl_omega_asym)
        print()
        print(rho_vec_list)
        print()
        print(qd_vel)
        print()
        print(cable_len_list)'''

    def sim_subscriber(self):
        print("#########################")
        print("publishing controls...")
        cont = controller()
        self.controller_setup(self.pl_params)
        F_list, M_list = cont.cooperative_suspended_payload_controller(self.pl, self.qd, self.pl_params, self.quad_params)

        for i in range(self.pl_params.nquad):
            FM_message = self.assembly_FM_message(F_list, M_list, i)
            self.FM_pub[i].publish(FM_message)

            #RPM_message_name = '/dragonfly' + str(i+1) + "/rpm_cmd"
            #RPM_pub = rospy.Publisher(RPM_message_name, PositionCommand)
            #RPM_message = assembly_RPM_message(, i)
            #RPM_pub.publish(RPM_message)

    '''
    def assembly_RPM_message(self, M_list):
        RPM_message = RPMCommand()

        return RPM_message
    '''


def main():
    controller_node()


if __name__ == '__main__':
	main()
        
