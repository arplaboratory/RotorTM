#! /usr/bin/env python3
import traj
import rospy
from rospy import Time
import numpy as np
import map
import create_options
# need to build quadrotor_msgs package
from rotor_tm_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from rotor_tm_traj.srv import Circle, Line


class traj_node:
	# traj_node class is used to init each kind of traj

	def __init__(self):
		self.radius = None
		self.period = None
		self.duration = None
		self.payload_start = None
		self.time_reference = None
		self.map = map.map()
		self.map.load_map()
		self.path = None
		self.curr_pose = np.append(np.zeros(3),np.array([1,0,0,0]))
		self.traj_start = False 


		## ROS Subscriber 
		rospy.Subscriber('payload/odom', Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)

		## ROS Publisher
		self.des_traj_pub = rospy.Publisher('payload/des_traj', PositionCommand, queue_size=1, tcp_nodelay=True)

        ## ROS Server
		server = []
		server.append(rospy.Service(node_name + '/Circle', Circle, self.circle_traj_cb))
		server.append(rospy.Service(node_name + '/Line', Line, self.line_traj_cb))
		server.append(rospy.Service(node_name + '/Min_Derivative_Line', Line, self.min_derivative_line_traj_cb))

		print("Trajectory Generator Initialization Finished")
	
	def circle_traj_cb(self, req):
		## call circular traj services
		self.current_traj = traj.traj()
		self.time_reference = rospy.get_time()
		self.traj_type = 1
		self.current_traj.circle(0, self.curr_state[0:3], req.radius, req.T, req.duration)
		self.traj_start = True

	def line_traj_cb(self, req):
		## call quintic line traj services
		path = [[self.curr_state[0],self.curr_state[1],self.curr_state[2]]]
		for pt_idx in range(len(req.path)):
			pt = req.path[pt_idx]
			path.append([pt.x,pt.y,pt.z])
		
		self.current_traj = traj.traj()
		self.time_reference = rospy.get_time()
		self.traj_type = 2
		self.current_traj.line_quintic_traj(0, self.map, np.array(path))
		self.traj_start = True

	def min_derivative_line_traj_cb(self, req):
		## call minimum derivative traj services
		path = [[self.curr_state[0],self.curr_state[1],self.curr_state[2]]]
		for pt_idx in range(len(req.path)):
			pt = req.path[pt_idx]
			path.append([pt.x,pt.y,pt.z])
		
		path = np.array(path)
		traj_constant = create_options.options()
		traj_constant.create_default_option(path.shape[0])
		self.current_traj = traj.traj()
		self.time_reference = rospy.get_time()
		self.traj_type = 3
		self.current_traj.min_snap_traj_generator(self, path, options=traj_constant)
		self.traj_start = True

	def odom_callback(self, data):

		self.curr_state = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,\
									data.pose.pose.orientation.w, data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])

	def send_des_traj(self,t):
		if self.traj_start:
			if (self.traj_type == 1):
				self.current_traj.circle(t-self.time_reference)
			elif (self.traj_type == 2):
				self.current_traj.line_quintic_traj(t-self.time_reference)
			else: 
				self.current_traj.min_snap_traj_generator(t-self.time_reference)
			
			# Publish the command
			now = rospy.get_rostime()
			message = PositionCommand()
			message.header.stamp.secs = now.secs
			message.header.stamp.nsecs = now.nsecs
			message.position.x=self.current_traj.state_struct["pos_des"][0]
			message.position.y=self.current_traj.state_struct["pos_des"][1]
			message.position.z=self.current_traj.state_struct["pos_des"][2]
			message.velocity.x=self.current_traj.state_struct["vel_des"][0]
			message.velocity.y=self.current_traj.state_struct["vel_des"][1]
			message.velocity.z=self.current_traj.state_struct["vel_des"][2]
			message.acceleration.x=self.current_traj.state_struct["acc_des"][0]
			message.acceleration.y=self.current_traj.state_struct["acc_des"][1]
			message.acceleration.z=self.current_traj.state_struct["acc_des"][2]
			message.jerk.x=self.current_traj.state_struct["jrk_des"][0]
			message.jerk.y=self.current_traj.state_struct["jrk_des"][1]
			message.jerk.z=self.current_traj.state_struct["jrk_des"][2]
			message.quaternion.w=self.current_traj.state_struct["quat_des"][0]
			message.quaternion.x=self.current_traj.state_struct["quat_des"][1]
			message.quaternion.y=self.current_traj.state_struct["quat_des"][2]
			message.quaternion.z=self.current_traj.state_struct["quat_des"][3]
			message.angular_velocity.x=self.current_traj.state_struct["omega_des"][0]
			message.angular_velocity.y=self.current_traj.state_struct["omega_des"][1]
			message.angular_velocity.z=self.current_traj.state_struct["omega_des"][2]
			self.des_traj_pub.publish(message)

def main():
	traj_node()

if __name__ == '__main__':
	## create a node called 'traj_node'
	node_name = 'traj_generator'
	rospy.init_node(node_name)
	rate = rospy.Rate(100)
	traj_node = traj_node()
	while not rospy.is_shutdown():
		t = rospy.get_time()
		traj_node.send_des_traj(t)
		rate.sleep()


        
