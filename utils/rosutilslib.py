import rospy
import numpy as np

def init_marker_msg(marker_msg, marker_type, action, frame_id, scale = [1,1,1], color = [1,0,0,0], mesh_resource=''):

    marker_msg.header.stamp = rospy.get_rostime() 

    marker_msg.header.frame_id = frame_id
    marker_msg.type = int(marker_type)
    marker_msg.action = int(action)
    marker_msg.scale.x = scale[0]
    marker_msg.scale.y = scale[1]
    marker_msg.scale.z = scale[2]
    marker_msg.color.a = color[0]
    marker_msg.color.r = color[1]
    marker_msg.color.g = color[2]
    marker_msg.color.b = color[3]
    marker_msg.mesh_resource = mesh_resource

    return marker_msg

def update_marker_msg(marker_stamped_msg, position, orientation, marker_id = 0):

    marker_stamped_msg.header.stamp = rospy.get_rostime() 

    marker_stamped_msg.pose.position.x = position[0]
    marker_stamped_msg.pose.position.y = position[1]
    marker_stamped_msg.pose.position.z = position[2]
    marker_stamped_msg.pose.orientation.w = orientation[0]
    marker_stamped_msg.pose.orientation.x = orientation[1]
    marker_stamped_msg.pose.orientation.y = orientation[2]
    marker_stamped_msg.pose.orientation.z = orientation[3]

    marker_stamped_msg.id = int(marker_id)
    return marker_stamped_msg

def update_odometry_msg(odometry_stamped_msg, position = np.array([0,0,0]), orientation = np.array([1,0,0,0]), linear_velocity = np.array([0,0,0]), angular_velocity = np.array([0,0,0])):

    odometry_stamped_msg.header.stamp = rospy.get_rostime()

    odometry_stamped_msg.pose.pose.position.x = position[0]
    odometry_stamped_msg.pose.pose.position.y = position[1]
    odometry_stamped_msg.pose.pose.position.z = position[2]
    odometry_stamped_msg.twist.twist.linear.x = linear_velocity[0]
    odometry_stamped_msg.twist.twist.linear.y = linear_velocity[1]
    odometry_stamped_msg.twist.twist.linear.z = linear_velocity[2]
    odometry_stamped_msg.pose.pose.orientation.w = orientation[0]
    odometry_stamped_msg.pose.pose.orientation.x = orientation[1]
    odometry_stamped_msg.pose.pose.orientation.y = orientation[2]
    odometry_stamped_msg.pose.pose.orientation.z = orientation[3]
    odometry_stamped_msg.twist.twist.angular.x = angular_velocity[0]
    odometry_stamped_msg.twist.twist.angular.y = angular_velocity[1]
    odometry_stamped_msg.twist.twist.angular.z = angular_velocity[2]
    
    return odometry_stamped_msg