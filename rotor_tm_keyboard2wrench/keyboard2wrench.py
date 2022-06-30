#!/usr/bin/env python

from __future__ import print_function
# from os import RWF_NOWAIT

import threading
import numpy as np
import rospy

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped

import sys, select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


WrenchMsg = Wrench

msg = """
Reading from the keyboard and Publishing to Wrench!
---------------------------
Increase Force:
   1,2,3,4,5,6
Decrease Force:
   q,w,e,r,t,y

CTRL-C to quit
"""

moveBindings = {
        '1':(0.025,0,0,0,0,0),
        '2':(0,0.025,0,0,0,0),
        '3':(0,0,0.025,0,0,0),
        '4':(0,0,0,0.01,0,0),
        '5':(0,0,0,0,0.01,0),
        '6':(0,0,0,0,0,0.01),
        'q':(-0.025,0,0,0,0,0),
        'w':(0,-0.025,0,0,0,0),
        'e':(0,0,-0.025,0,0,0),
        'r':(0,0,0,-0.01,0,0),
        't':(0,0,0,0,-0.01,0),
        'y':(0,0,0,0,0,-0.01),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/payload/so3_control/ext_wrench', WrenchMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.speed = 1
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, yaw, roll, pitch):
        self.condition.acquire()
        self.x += x
        self.y += y
        self.z += z
        self.yaw += yaw
        self.roll += roll
        self.pitch += pitch
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        wrench_msg = WrenchMsg()

        if stamped:
            wrench = wrench_msg.wrench
            wrench_msg.header.stamp = rospy.Time.now()
            wrench_msg.header.frame_id = twist_frame
        else:
            wrench = wrench_msg
        while not self.done:
            if stamped:
                wrench_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            wrench.force.x = self.x * self.speed
            wrench.force.y = self.y * self.speed
            wrench.force.z = self.z * self.speed
            wrench.torque.x = self.yaw * self.speed
            wrench.torque.y = self.roll * self.speed
            wrench.torque.z = self.pitch * self.speed
            self.condition.release()

            # Publish.
            self.publisher.publish(wrench_msg)

        # Publish stop message when thread exits.
        wrench.force.x = 0
        wrench.force.y = 0
        wrench.force.z = 0
        wrench.torque.x = 0
        wrench.torque.y = 0
        wrench.torque.z = 0
        self.publisher.publish(wrench_msg)

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def check(x, y, z, yaw, roll, pitch):
    return "x %s\ty %s \tz %s\troll %s \tpitch %s \tyaw %s" % (np.around(x,3), np.around(y,3), np.around(z,3), np.around(yaw,3), np.around(roll,3), np.around(pitch,3))

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_wrench_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 100)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = WrenchStamped
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    yaw = 0
    roll = 0
    pitch = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, yaw, roll, pitch)

        print(msg)

        while(1):
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                yaw = moveBindings[key][3]
                roll = moveBindings[key][4]
                pitch = moveBindings[key][5]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]


                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, yaw, roll, pitch)
            print(check(pub_thread.x, pub_thread.y, pub_thread.z, pub_thread.yaw, pub_thread.roll, pub_thread.pitch))

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

