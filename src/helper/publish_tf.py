#!/usr/bin/env python
import egm_pb2, egm_helper, helper
from time import sleep
import rospy
import math
import numpy as np
import time, datetime
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
import subprocess, sys, os
import tf

def publish_tf(parent, child, topic, listener):
    (trans, quat) = listener.lookupTransform(parent, child, rospy.Time(0))
    pose = trans+quat
    pose_msg = Float32MultiArray()
    pose_msg.data = pose
    topic.publish(pose_msg)

if __name__=='__main__':
    rospy.init_node('publish_tf')
    listener = tf.TransformListener()
    rospy.sleep(1.)
    #define Publishers
    viewer_frame_pub=rospy.Publisher('/frame/viewer_frame_pub', Float32MultiArray, queue_size = 10, latch=True)
    track_start_frame_pub=rospy.Publisher('/frame/track_start_frame_pub', Float32MultiArray, queue_size = 10, latch=True)
    vicon_world_frame_pub=rospy.Publisher('/frame/vicon_world_frame_pub', Float32MultiArray, queue_size = 10, latch=True)
    vicon_world_frame_pub=rospy.Publisher('/frame/vicon_world_frame_pub', Float32MultiArray, queue_size = 10, latch=True)
    #publish static frames
    publish_tf("map", "viewer_rgb_optical_frame", viewer_frame_pub, listener)
    publish_tf("map", "track_start", track_start_frame_pub, listener)
    publish_tf("map", "viconworld", vicon_world_frame_pub, listener)
    rospy.spin()




    # helper.terminate_ros_node('/record')
