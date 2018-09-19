#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
import tf

def publish_object_pose(listener):
    object_pose_pub=rospy.Publisher('/object_pose', Float32MultiArray, queue_size = 10, latch=True)

    while True:
        (trans, quat) = listener.lookupTransform("track_start", "/vicon/StainlessSteel/StainlessSteel_rect", rospy.Time(0))
        rpy = tf.transformations.euler_from_quaternion(quat)
        theta = rpy[2]
        object_pose_msg = Float32MultiArray()
        object_pose_msg.data = [trans[0], trans[1], theta]
        object_pose_pub.publish(object_pose_msg)
        rospy.Rate(250).sleep()
        # print object_pose_msg

def publish_object_pose_world(listener):
    object_pose_pub=rospy.Publisher('/object_pose', Float32MultiArray, queue_size = 10, latch=True)

    while True:
        (trans, quat) = listener.lookupTransform("map", "/vicon/StainlessSteel/StainlessSteel_rect", rospy.Time(0))
        rpy = tf.transformations.euler_from_quaternion(quat)
        theta = rpy[2]
        object_pose_msg = Float32MultiArray()
        object_pose_msg.data = [trans[0], trans[1], theta]
        object_pose_pub.publish(object_pose_msg)
        rospy.Rate(250).sleep()
        # print object_pose_msg

if __name__ == '__main__':
    rospy.init_node('vicon_publisher', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(1)
    publish_object_pose(listener)
