#!/usr/bin/env python
import time
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np

class ROSHelper():

    def __init__(self):
        self.command_pose_sub = rospy.Subscriber("/command_pose", PoseStamped, self.load_command_pose)
        self.joint_state_pub = rospy.Publisher("/joint_state", JointState, queue_size = 2)
        self.sent_pose_pub = rospy.Publisher("/sent_pose", PoseStamped, queue_size = 2)
        self.measured_pose_pub = rospy.Publisher("/measured_pose", PoseStamped, queue_size = 2)
        self.command_pose = PoseStamped()

    def load_command_pose(self, data):
        self.command_pose = data

    def get_command_pose(self):
        return self.command_pose

    def publish_joint_state(self, joints):
        self.joint_state_pub.publish(joints)

    def publish_measured_pose(self, pose):
        self.measured_pose_pub.publish(pose)

    def publish_sent_pose(self, pose):
        self.sent_pose_pub.publish(pose)
