#!/usr/bin/env python
import time
import rospy
import numpy as np
import subprocess, sys, os
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import ROSHelper, RobotController
import cProfile

# Initial position of the end-effector (check RAPID code)
initial_pos = [-0.045, 0.009, 0.1782]
# Coordinate limits (1: lower bound, 2: upper bound)
x_limits = [0.1, 0.6]
y_limits = [-0.4, 0.4]
z_limits = [0, 0.5]
# Rate
hz = 250

def main():
    rospy.init_node('EGMControl')

    # Initializing helper instances
    ros_helper = ROSHelper.ROSHelper()
    robot_controller = RobotController.RobotController(initial_pos, x_limits, y_limits, z_limits)

    rospy.set_param('egm_status', True)

    # Wait until robot reaches initial position (see RAPID code)
    rospy.sleep(1)
    rospy.loginfo('[EGMControl] Ready')

    time_start = rospy.get_time()
    rate = rospy.Rate(hz)

    while rospy.get_param('egm_status') == True:
        command_pose = ros_helper.get_command_pose()
        # egm_mode can be: 1 (velocity) or any other/not defined (pose)
        if rospy.get_param('egm_mode', 0) == 1:
            robot_controller.set_robot_velocity(command_pose, hz)
        else:
            robot_controller.set_robot_pose(command_pose)

        feedback = robot_controller.get_robot_feedback()
        joints = feedback.joints.joints
        pos = feedback.cartesian.pos
        quaternion = feedback.cartesian.orient
        ros_helper.publish_joint_state(joints)
        ros_helper.publish_measured_pose(pos, quaternion)
        rate.sleep()

    # Deleting instances to free resources
    del ros_helper
    del robot_controller
    rospy.loginfo('[EGMControl] End of program')


if __name__=='__main__':
    cProfile.run('main()')
