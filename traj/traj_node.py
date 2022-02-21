#! /usr/bin/env python

import traj
import rospy
from rospy import Time
import numpy as np
import map
import create_options
# need to build quadrotor_msgs package
from quadrotor_msgs.msg import PositionCommand


class traj_node:
	# traj_node class is used to init each kind of traj

	def __init__(self):
		self.radius = None
		self.period = None
		self.duration = None
		self.payload_start = None
		self.time_reference = None
		self.map = None
		self.path = None
	
	def circular_traj_init(self, radius = 1.0, period = 6.0, duration = 6.0, payload_start = np.array([[0.0], [0.0], [0.0], [1.0], [0.0], [0.0], [0.0]])):
		## initialize circular traj
		self.radius = radius
		self.period = period
		self.duration = duration
		self.payload_start = payload_start
		circular_traj = traj.traj()
		self.time_reference = rospy.get_time()
		circular_traj.circle(0, init_pos=self.payload_start[0:3,:], r=self.radius, period=self.period, circle_duration=self.duration)
		return circular_traj

	def line_traj_init(self, specified_map = None, path = np.array([[0., 0., 0.], [0.5, -0.5, 0.25], [1.0, 0.0, 0.5], [1.5, -0.5, 0.75], [-2.0, 0.0, 1.0], [1.0, 3.0, 0.5], [-1.0, 3.0, 0.5], [1.0, 3.0, 0.5], [-1.0, 3.0, 0.5]])):
		## initialize line traj
		self.map = map.map()
		self.map.load_map()
		self.path = path
		line_traj = traj.traj()
		self.time_reference = rospy.get_time()
		line_traj.line_quintic_traj(0, map=self.map, path=self.path)
		return line_traj

	def min_snap_traj_init(self, path = np.array([[0.0,0.0,0.0],[0.5,-0.5,0.25],[1.0,0.0,0.5],[1.5,-0.5,0.75],[2.0,0.0,1.0]]), options = None):
		traj_constant = create_options.options()
		traj_constant.create_default_option(path.shape[0])
		snap_traj = traj.traj()
		self.time_reference = rospy.get_time()
		snap_traj.min_snap_traj_generator(self, path=path, options=traj_constant)
		return snap_traj

def callback(data, arg):
	t = rospy.get_time()
	now = rospy.get_rostime()
	traj_item = arg[0]
	traj = arg[1]
	check = arg[2]
	if (check==1):
		traj_item.circle(t-traj.time_reference)
	elif (check==2):
		traj_item.line_quintic_traj(t-traj.time_reference)
	else: 
		traj_item.min_snap_traj_generator(t-traj.time_reference)

	pub = rospy.Publisher('des_traj', PositionCommand)

	message = PositionCommand()
	message.header.stamp.secs = now.secs
	message.header.stamp.nsecs = now.nsecs
	message.position.x=traj_item.state_struct["pos_des"][0]
	message.position.y=traj_item.state_struct["pos_des"][1]
	message.position.z=traj_item.state_struct["pos_des"][2]
	message.velocity.x=traj_item.state_struct["vel_des"][0]
	message.velocity.y=traj_item.state_struct["vel_des"][1]
	message.velocity.z=traj_item.state_struct["vel_des"][2]
	message.acceleration.x=traj_item.state_struct["acc_des"][0]
	message.acceleration.y=traj_item.state_struct["acc_des"][1]
	message.acceleration.z=traj_item.state_struct["acc_des"][2]
	message.jerk.x=traj_item.state_struct["jrk_des"][0]
	message.jerk.y=traj_item.state_struct["jrk_des"][1]
	message.jerk.z=traj_item.state_struct["jrk_des"][2]
	message.yaw=traj_item.state_struct["qd_yaw_des"]
	message.yaw_dot=traj_item.state_struct["qd_yawdot_des"]
	pub.publish(message)

# chmod +x traj_node.py
# under workspace, rosrun pacakge_name traj_node.py
# run after simulation is running
# min_snap_traj still needs work

def main():
	## create a node called 'traj_node'
	rospy.init_node('traj_node')
	
	## init a traj
	# 1 -> circle traj
	# 2 -> line traj
	# 3 -> min snap traj
	traj_type = 3
	
	traj = traj_node()
	if (traj_type == 1):
		traj_item = traj.circular_traj_init()
	elif (traj_type == 2):
		traj_item = traj.line_traj_init()
	else:
		traj_item = traj.min_snap_traj_init()

	## subscribe to simulation publication, publishing trajectory using callback function
	rospy.Subscriber('/Enter Name', PositionCommand, callback, (traj_item, traj, traj_type))
	rospy.spin()

if __name__ == '__main__':
	main()
        
