#!/usr/bin/env python
import time
import rospy
import numpy as np
import subprocess, sys, os
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import helpers.EGMHelper as EGMHelper
import ROSHelper, RobotController
import cProfile

# Coordinate limits (1: lower bound, 2: upper bound)
x_limits = [100.0, 600.0]
y_limits = [-400.0, 400.0]
z_limits = [0.0, 500.0]
# Rate
hz = 250.0

def main():
    rospy.init_node('EGMControl')

    # Initializing helper instances
    ros_helper = ROSHelper.ROSHelper()
    robot_controller = RobotController.RobotController(x_limits, y_limits, z_limits)

    rospy.loginfo('[EGMControl] Ready')
    rospy.sleep(1/hz)

    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        # Getting command from ROS and sending it to the robot
        command_pose = ros_helper.get_command_pose()
        command_mode = rospy.get_param('egm_mode', 'position')
        sent_pose = robot_controller.send_command(command_pose, command_mode, hz)
        ros_helper.publish_sent_pose(sent_pose)

        # Getting feedback from the robot and sending it back to ROS
        measured_pose, joint_state = robot_controller.get_robot_feedback()
        ros_helper.publish_measured_pose(measured_pose)
        ros_helper.publish_joint_state(joint_state)

        rate.sleep()

    # Deleting instances to free resources
    del ros_helper
    del robot_controller
    rospy.loginfo('[EGMControl] End of program')


if __name__=='__main__':
    main()
