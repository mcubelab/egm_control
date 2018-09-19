#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
import geometry_msgs
import json
import helper

object_pose = []
buffer_len = 2500
x_path_buffer = []
y_path_buffer = []

def callback(data):
    global object_pose, object_pose_buffer
    object_pose = data.data
    x_path_buffer.append(object_pose[0])
    y_path_buffer.append(object_pose[1])

if __name__ == '__main__':
    rospy.init_node('marker_object_trajectory', anonymous=True)
    object_pos_pub = rospy.Subscriber("/object_pose", Float32MultiArray, callback)
    marker_pub = rospy.Publisher('/visualization_marker_trajectory', Marker, queue_size = 10)
    rate = 30
    rospy.sleep(3.)

    while True:
        if len(x_path_buffer)>buffer_len:
            # print len(x_path_buffer)
            x_path_buffer = x_path_buffer[-buffer_len:-1]
            y_path_buffer = y_path_buffer[-buffer_len:-1]
        #
        # if len(x_path_buffer)>150:
        #     x_path = np.array(x_path_buffer[0:-150])
        #     y_path = np.array(y_path_buffer[0:-150])
        #     z_path = y_path*0
        # else:
        x_path = np.array(x_path_buffer)
        y_path = np.array(y_path_buffer)
        z_path = y_path*0
        # print x_path[-1], y_path[-1], z_path[-1]
        # import pdb;pdb.set_trace()
        # print x_path
        line_strip = helper.trajectory_viz(x_path, y_path, z_path + 0.015, line_thick = 0.002, color = (0., 0, 1., 1.))
        marker_pub.publish(line_strip)
        rospy.Rate(rate).sleep()
