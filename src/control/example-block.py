#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# All sizes in mm!
hz = 248.0 # hz
config = [
    [0.0, 5.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [-60.0, 5.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [5.0, 10.0],
    [0.0, 3.0],
    [-40.0, 5.0]
]

if __name__ == '__main__':
    rospy.init_node('exampleblock', anonymous=True)
    command_pose_pub = rospy.Publisher('/command_pose', PoseStamped, queue_size = 100, latch=True)

    rospy.set_param('egm_mode', 'velocity')

    rate = rospy.Rate(hz)
    start_time = rospy.Time.now().to_sec()
    i = 0

    while (not rospy.is_shutdown()) and i < len(config):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        # Position in mm or velocity in mm/s
        pose.pose.position.x = config[i][0]
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        # Orientation or angular velocity in xyzw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        command_pose_pub.publish(pose)

        if rospy.Time.now().to_sec()-start_time >= config[i][1]:
            i += 1
            start_time = rospy.Time.now().to_sec()

        rate.sleep()
