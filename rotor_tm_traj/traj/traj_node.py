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
from std_msgs.msg import Header
from rotor_tm_traj.srv import Circle, Line, CircleWithRotation, StepPose
from rotor_tm_traj.srv import CircleResponse, LineResponse, CircleWithRotationResponse, StepPoseResponse
from scipy.spatial.transform import Rotation as rot


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
		self.is_finished = True


		## ROS Subscriber 
		rospy.Subscriber('payload/odom', Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)

		## ROS Publisher
		self.des_traj_pub = rospy.Publisher('payload/des_traj', PositionCommand, queue_size=1, tcp_nodelay=True)
		self.cir_traj_status_pub = rospy.Publisher('payload/cir_traj_status', Header, queue_size=1, tcp_nodelay=True)
		self.line_traj_status_pub = rospy.Publisher('payload/line_traj_status', Header, queue_size=1, tcp_nodelay=True)
		self.min_der_traj_status_pub = rospy.Publisher('payload/min_der_traj_status', Header, queue_size=1, tcp_nodelay=True)
		self.step_traj_status_pub = rospy.Publisher('payload/step_input_status', Header, queue_size=1, tcp_nodelay=True)

        ## ROS Server
		server = []
		server.append(rospy.Service(node_name + '/Circle', Circle, self.circle_traj_cb))
		server.append(rospy.Service(node_name + '/Circle_Rot_Rigidbody', CircleWithRotation, self.circle_with_rot_body_traj_cb))
		server.append(rospy.Service(node_name + '/Line', Line, self.line_traj_cb))
		server.append(rospy.Service(node_name + '/Min_Derivative_Line', Line, self.min_derivative_line_traj_cb))
		server.append(rospy.Service(node_name + '/Step_Pose', StepPose, self.step_input_cb))

		print("Trajectory Generator Initialization Finished")

	def circle_traj_cb(self, req):
	# DESCRIPTION
	# call back function for the circular trajectory service "/Circle"
	# It first check if (if any) previous trajectory generation has finished
	# if yes, it will create a circlar traj object that would initialize the trajectory (when called the first time)
	# 												          output waypoint of the trajectory (for subsequent calls)
	
	# INPUT
	# req		- parameters necessary for create the circular trajectory. 
	# 			  Check ~/ws/src/rotorTM/rotor_tm_traj/srv/Circle.srv for details

	# OUTPUT
	# /			- update the attribute self.current_traj to be of circular type
		if self.traj_start:
			self.is_finished = self.current_traj.finished
		## call circular traj services
		if self.is_finished == False:
			print("Please wait for the previous traj to finish")
		else:
			self.current_traj = traj.traj()
			self.current_traj.circle(0, self.curr_state[0:3], req.radius, req.T, req.duration)
			self.time_reference = rospy.get_time()
			self.traj_start = True
			status_msgs = Header()
			status_msgs.stamp = rospy.get_rostime()
			self.cir_traj_status_pub.publish(status_msgs)
			
		response = CircleResponse()
		response.success = True
		response.message = "Circle Trajectory Generated"
		return response	
	
	def circle_with_rot_body_traj_cb(self, req):
	# DESCRIPTION
	# call back function for the circular trajectory service "/CircleWithRotBody"
	# It first check if (if any) previous trajectory generation has finished
	# if yes, it will create a circlar traj object that would initialize the trajectory (when called the first time)
	# 												          output waypoint of the trajectory (for subsequent calls)
	
	# INPUT
	# req		- parameters necessary for create the circular trajectory. 
	# 			  Check ~/ws/src/rotorTM/rotor_tm_traj/srv/Circle_Rot_Rigidbody.srv for details

	# OUTPUT
	# /			- update the attribute self.current_traj to be of circular type
		if self.traj_start:
			self.is_finished = self.current_traj.finished
		## call circular traj services
		if self.is_finished == False:
			print("Please wait for the previous traj to finish")
		else:
			angle_amp = [req.angle_amp[0],  req.angle_amp[1],  req.angle_amp[2]]
			self.current_traj = traj.traj()
			self.current_traj.circlewithrotbody(0, self.curr_state[0:7], req.radius, angle_amp, req.T, req.duration)
			self.time_reference = rospy.get_time()
			self.traj_start = True
			status_msgs = Header()
			status_msgs.stamp = rospy.get_rostime()
			self.cir_traj_status_pub.publish(status_msgs)

		response = CircleWithRotationResponse() 
		response.success = True
		response.message = "Circle Trajectory with Rotation Generated"
		return response	

	def line_traj_cb(self, req):
	# DESCRIPTION
	# call back function for the line trajectory service "/Line"
	# It first check if (if any) previous trajectory generation has finished
	# if yes, it will create a line traj object that would initialize the trajectory (when called the first time)
	# 												       output waypoint of the trajectory (for subsequent calls)
	
	# INPUT
	# req		- parameters necessary for create the line trajectory. 
	# 			  Check ~/ws/src/rotorTM/rotor_tm_traj/srv/Line.srv for details

	# OUTPUT
	# /			- update the attribute self.current_traj to be of line type
		if self.traj_start:
			self.is_finished = self.current_traj.finished
		if self.is_finished == False:
			print("Please wait for the previous traj to finish")
		else:
			## call quintic line traj services
			path = [[self.curr_state[0],self.curr_state[1],self.curr_state[2]]]
			for pt_idx in range(len(req.path)):
				pt = req.path[pt_idx]
				path.append([pt.x,pt.y,pt.z])
			
			self.current_traj = traj.traj()
			self.current_traj.line_quintic_traj(0, self.map, np.array(path))
			self.time_reference = rospy.get_time()
			self.traj_start = True
			status_msgs = Header()
			status_msgs.stamp = rospy.get_rostime()
			self.line_traj_status_pub.publish(status_msgs)

		response = LineResponse() 
		response.success = True
		response.message = "Line Trajectory with Rotation Generated"
		return response	

	def min_derivative_line_traj_cb(self, req):
	# DESCRIPTION
	# call back function for the trajectory service "/Min_Derivative_Line"
	# It first check if (if any) previous trajectory generation has finished
	# if yes, it will create a lineMin_Derivative_Line traj object that would initialize the trajectory (when called the first time)
	# 												       					  output waypoint of the trajectory (for subsequent calls)
	
	# INPUT
	# req		- parameters necessary for create the lineMin_Derivative_Linse trajectory. 
	# 			  Check ~/ws/src/rotorTM/rotor_tm_traj/srv/Line.srv for details

	# OUTPUT
	# /			- update the attribute self.current_traj to be of lineMin_Derivative_Line type
		if self.traj_start:
			self.is_finished = self.current_traj.finished
		if self.is_finished == False:
			print("Please wait for the previous traj to finish")
		else:
			## call minimum derivative traj services
			path = [[self.curr_state[0],self.curr_state[1],self.curr_state[2]]]
			for pt_idx in range(len(req.path)):
				pt = req.path[pt_idx]
				path.append([pt.x,pt.y,pt.z])
			path = np.array(path)
			traj_constant = create_options.options()
			traj_constant.create_default_option(path.shape[0])
			self.current_traj = traj.traj()
			self.current_traj.min_snap_traj_generator(self, path, options=traj_constant)
			self.time_reference = rospy.get_time()
			self.traj_start = True
			status_msgs = Header()
			status_msgs.stamp = rospy.get_rostime()
			self.min_der_traj_status_pub.publish(status_msgs)

		response = LineResponse() 
		response.success = True
		response.message = "Line Trajectory with Rotation Generated"
		return response	

	def step_input_cb(self, req):
	# DESCRIPTION
	# call back function for the step input service "/Stepinput"
	# It first check if (if any) previous trajectory generation has finished
	# if yes, it will create a step input object that would initialize the trajectory (when called the first time)
	# 												          output waypoint of the trajectory (for subsequent calls)
	
	# INPUT
	# req		- parameters necessary for create the step input. 
	# 			  Check ~/ws/src/rotorTM/rotor_tm_traj/srv/Step.srv for details

	# OUTPUT
	# /			- update the attribute self.current_traj to be of step input type
		if self.traj_start:
			self.is_finished = self.current_traj.finished

		## call step input services
		if self.is_finished == False:
			print("Please wait for the previous traj to finish")
		else:
			quat = rot.from_euler('ZYX', [req.yaw,req.pitch,req.roll],degrees=True).as_quat() 
			pose = np.array([req.position[0],req.position[1],req.position[2],quat[3],quat[0],quat[1],quat[2]]) 
			self.current_traj = traj.traj()
			self.current_traj.step_des(0, pose)
			self.time_reference = rospy.get_time()
			self.traj_start = True
			status_msgs = Header()
			status_msgs.stamp = rospy.get_rostime()
			self.step_traj_status_pub.publish(status_msgs)

		response = StepPoseResponse() 
		response.success = True
		response.message = "Step Pose Called"
		return response	

	def odom_callback(self, data):
	# DESCRIPTION
	# call back function for payload/odom subscriber. Extract current state
	
	# INPUT
	# data		- nav_msgs.msg, Odometry type message. Acquired from odom callback

	# OUTPUT
	# /			- update the attribute self.current_state
		self.curr_state = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z,\
									data.pose.pose.orientation.w, data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z])
		t = rospy.get_time()
		self.send_des_traj(t)

	def send_des_traj(self,t):
	# DESCRIPTION
	# when called, will spit out a new waypoint for the current trajectory type. 
	# Must be call after initialization of each type of trajectory
	
	# INPUT
	# t  		- float, current simulation time

	# OUTPUT
	# /			- publish PositionCommand type message to payload/des_traj
		if self.traj_start:
			if (self.current_traj.traj_type != 0):
				if (self.current_traj.traj_type == 1):
					self.current_traj.circle(t-self.time_reference)
					if not self.current_traj.finished:
						status_msgs = Header()
						status_msgs.stamp = rospy.get_rostime()
						self.cir_traj_status_pub.publish(status_msgs)
				elif (self.current_traj.traj_type == 2):
					self.current_traj.circlewithrotbody(t-self.time_reference)
					if not self.current_traj.finished:
						status_msgs = Header()
						status_msgs.stamp = rospy.get_rostime()
						self.cir_traj_status_pub.publish(status_msgs)
				elif (self.current_traj.traj_type == 3):
					self.current_traj.line_quintic_traj(t-self.time_reference)
					if not self.current_traj.finished:
						status_msgs = Header()
						status_msgs.stamp = rospy.get_rostime()
						self.line_traj_status_pub.publish(status_msgs)
				elif (self.current_traj.traj_type == 4): 
					self.current_traj.min_snap_traj_generator(t-self.time_reference)
					if not self.current_traj.finished:
						status_msgs = Header()
						status_msgs.stamp = rospy.get_rostime()
						self.min_der_traj_status_pub.publish(status_msgs)
				elif (self.current_traj.traj_type == 5): 
					self.current_traj.step_des(t-self.time_reference)
					if not self.current_traj.finished:
						status_msgs = Header()
						status_msgs.stamp = rospy.get_rostime()
						self.step_traj_status_pub.publish(status_msgs)
			
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

	# at a constant rate of 75 Hz, compute new trajectory way points
	rate = rospy.Rate(100)
	traj_node = traj_node()
	while not rospy.is_shutdown():
		t = rospy.get_time()
		#traj_node.send_des_traj(t)
		rate.sleep()


        
