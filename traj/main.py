#! /usr/bin/env python

from tf import TransformBroadcaster
import traj
import rospy
from rospy import Time
import numpy as np
import map

def main():
	# init circular traj
	radius = 1.0
	period = 6.0
	duration = 6.0
	payload_start = np.array([[0.0], [0.0], [0.0], [1.0], [0.0], [0.0], [0.0]])
	
	# init line traj
	map0 = map.map()
	map0.load_map()
	path1 = np.array([[0., 0., 0.], [0.5, -0.5, 0.25], [1.0, 0.0, 0.5], [1.5, -0.5, 0.75], [-2.0, 0.0, 1.0], [1.0, 3.0, 0.5], [-1.0, 3.0, 0.5], [1.0, 3.0, 0.5], [-1.0, 3.0, 0.5]])

	rospy.init_node('circle_traj_test')
	state_struct = {}
	b = TransformBroadcaster()


	circular_traj = traj.traj()
	circular_traj.circle(t=0, state_struct=state_struct, init_pos=payload_start[0:3,:], r=radius, period=period, circle_duration=duration)
	line_traj = traj.traj()
	line_traj.line_quintic_traj(t=0, map=map0, path=path1)

	rotation = (0.0, 0.0, 0.0, 1.0)
	rate = rospy.Rate(10) # 10 Hz
	t = 0

	while not rospy.is_shutdown():
		t += 0.1
		state_struct = circular_traj.circle(t, state_struct=state_struct)
		desired_state = line_traj.line_quintic_traj(t)
		translation1 = (state_struct["pos_des"][0], state_struct["pos_des"][1], state_struct["pos_des"][2])
		translation2 = (desired_state["pos"][0], desired_state["pos"][1], desired_state["pos"][2])
		b.sendTransform(translation1, rotation, Time.now(), 'test_robot_circle', '/map')
		b.sendTransform(translation2, rotation, Time.now(), 'test_robot_line', '/map')
		# publish circle and line traj to tf, use run rviz rviz to visualize (add tf to displays)
		print(translation1)
		print(translation2)
		print()
		rate.sleep()


if __name__ == '__main__':
	main()
        
