#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

pos = [-0.045, 0.009, 0.1782]
v = 0.01 # m/s
delta = 1
hz = 250

if __name__ == '__main__':
    rospy.init_node('example', anonymous=True)
    command_pose_pub = rospy.Publisher('/command_pose', PoseStamped, queue_size = 10, latch=True)

    rate = rospy.Rate(hz)
    rospy.sleep(1)

    rospy.set_param('egm_status_example', True)

    while rospy.get_param('egm_status_example') == True:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        if abs(pos[1]) >= 0.2:
            delta = -delta
        pos[1] += delta*v/hz
        # Position in meters
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        # Orientation in xyzw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        command_pose_pub.publish(pose)
        rate.sleep()
