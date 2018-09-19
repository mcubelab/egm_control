#!/usr/bin/env python
from helper import egm_pb2, egm_helper, helper
from time import sleep
import rospy
import math
import numpy as np
import time, datetime
import tf
import threading
from std_msgs.msg import Float32MultiArray, Float32
import subprocess, sys, os

robot_velocity = []

def callback(data):
    global robot_velocity
    robot_velocity = data.data

if __name__=='__main__':
    rospy.init_node('egm_control')
    listener = tf.TransformListener()
    #ROS Subscribers
    robot_velocity_pub = rospy.Subscriber("/robot_velocity", Float32MultiArray, callback)
    #ROS Publishers
    time_pub=rospy.Publisher('/time', Float32, queue_size = 10, latch=True)
    #Saving rosbag options
    name_of_bag  = str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
    topics = ["/time", "/cart_command_states", "/cart_sensed_states", "/joint_states", "/robot_velocity", "/object_pose"]
    dir_save_bagfile = os.environ['HOME'] + '/pushing_benchmark_data/'
    rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
    #Wait until robot reaches initial position
    rospy.sleep(1)
    rospy.loginfo('[EGM CONTROL] ready')
    #Initialize time
    helper.publish_time(0., time_pub)
    #initialize controller object
    EGM = egm_helper.EGMController(listener)
    #Define initial position of robot pusher
    EGM.position = np.array([-0.045, 0.009, .1782])
    #Control rate
    rate = 1000 # 250hz
    #start control loop
    rospy.set_param('is_exit', False)
    t_loop = 1/rate
    t_loop_start = time_ini = rospy.get_time()
    while rospy.get_param('is_exit')==False:
        #0. publish time
        helper.publish_time(rospy.get_time()-time_ini, time_pub)
        # 1. get robot pose and publish robot states (rviz)
        EGM.get_robot_pos()
        # send robot_velocity command
        t_loop = rospy.get_time() - t_loop_start
        t_loop_start = rospy.get_time()
        EGM.send_robot_vel(robot_velocity, t_loop)
        #sleep to maintain 1000hz
        rospy.Rate(rate).sleep()
    #close EGM socket connection and stop rosbag recording
    EGM.sock.close()
    helper.terminate_ros_node('/record')
    rospy.loginfo('[EGM CONTROL] End of program')
