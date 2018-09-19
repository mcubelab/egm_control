#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Header
from sensor_msgs.msg import JointState
sys.path.append('../../push_control/src')
sys.path.append('../../push_control')
from push_control.srv import *
import tf
from helper import vicon_publisher, egm_helper

if __name__ == '__main__':
    rospy.init_node('object_tracker', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(1)
    #Define rostopic publishers
    robot_velocity_pub=rospy.Publisher('/robot_velocity', Float32MultiArray, queue_size = 10, latch=True)
    time_pub=rospy.Publisher('/time', Float32, queue_size = 10, latch=True)
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size = 2, latch=True)
    cart_sensed_pub = rospy.Publisher("/cart_sensed_states", Float32MultiArray, queue_size = 2, latch=True)
    cart_command_pub = rospy.Publisher("/cart_command_states", Float32MultiArray, queue_size = 2, latch=True)
    joint_states_pub = rospy.Publisher("/joint_states", JointState, queue_size = 2)
    #Define rosservices
    # controller_srv = rospy.ServiceProxy('/push_control/srv_get_nominal', Nominal_SRV)
    #publish time
    time_msg = Float32()
    time_msg.data = 0;
    time_pub.publish(time_msg)
    #get nominal state
    # out = controller_srv(0)
    # xs = np.array(out.xs)
    # object_pose = xs[0:3]
    # pusher_pos = xs[3:5]
    # us = np.array(out.us)
    #publish robot cart
    # robot_cart_msg = Float32MultiArray()
    # robot_cart_msg.data = pusher_pos
    # cart_command_pub.publish(robot_cart_msg)
    # cart_sensed_pub.publish(robot_cart_msg)
    #publish robot joint states
    # joints = [0,0,0,0,0,0]
    # js = JointState()
    # js.header = Header()
    # js.header.stamp = rospy.Time.now()
    # js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    # js.position = [j for j in joints]
    # js.velocity = [0.0 for i in xrange(6)]
    # js.effort = [0.0 for i in xrange(6)]
    # joint_states_pub.publish(js)
    # #publish robot velocity
    # vel_msg = Float32MultiArray()
    # vel_msg.data = us
    # robot_velocity_pub.publish(vel_msg)
    #start tracking the object_poserospy.sleep(1)
    vicon_publisher.publish_object_pose(listener)
