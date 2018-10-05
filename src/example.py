#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# All sizes in mm!
hz = 250.0 # hz
t = float(sys.argv[2]) # s

if __name__ == '__main__':
    rospy.init_node('example', anonymous=True)
    command_pose_pub = rospy.Publisher('/command_pose', PoseStamped, queue_size = 100, latch=True)

    rospy.set_param('egm_mode', 'velocity')

    rate = rospy.Rate(hz)
    start_time = rospy.Time.now().to_sec()

    while (not rospy.is_shutdown()) and rospy.Time.now().to_sec()-start_time < t:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        # Position in mm or velocity in mm/s
        pose.pose.position.x = float(sys.argv[1])
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        # Orientation or angular velocity in xyzw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        command_pose_pub.publish(pose)
        rate.sleep()
