#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# All sizes in mm!
hz = 250.0

if __name__ == '__main__':
    rospy.init_node('example', anonymous=True)
    command_pose_pub = rospy.Publisher('/command_pose', PoseStamped, queue_size = 10, latch=True)

    rate = rospy.Rate(hz)
    rospy.sleep(1)

    rospy.set_param('egm_status_example', True)
    rospy.set_param('egm_mode', 1)

    while rospy.get_param('egm_status_example') == True:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        # Position in meters
        pose.pose.position.x = 10.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        # Orientation in xyzw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        command_pose_pub.publish(pose)
        rate.sleep()
