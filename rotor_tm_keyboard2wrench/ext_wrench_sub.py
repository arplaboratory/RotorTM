#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import TwistStamped

import sys, select

def callback(data):
    rospy.loginfo("x: %s \n", data.force.x)
    rospy.loginfo("y: %s \n", data.force.y)
    rospy.loginfo("z: %s \n", data.force.z)
    rospy.loginfo("pitch: %s \n", data.torque.x)
    rospy.loginfo("roll: %s \n", data.torque.y)
    rospy.loginfo("yaw: %s \n", data.torque.z)

    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/payload/so3_control/ext_wrench", Wrench, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()