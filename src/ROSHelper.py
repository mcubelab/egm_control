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
        self.joint_state_pub = rospy.Publisher("/joint_state", JointState, queue_size = 100)
        self.sent_pose_pub = rospy.Publisher("/sent_pose", PoseStamped, queue_size = 100)
        self.measured_pose_pub = rospy.Publisher("/measured_pose", PoseStamped, queue_size = 100)
        self.command_poses = []
        self.max_queued = 0

    def load_command_pose(self, data):
        self.command_poses.append(data)
        self.max_queued = max(len(self.command_poses), self.max_queued)

    def get_command_pose(self):
        if len(self.command_poses) > 0:
            data = self.command_poses[0]
            del self.command_poses[0]
            return data
        else:
            return None

    def publish_joint_state(self, joints):
        self.joint_state_pub.publish(joints)

    def publish_measured_pose(self, pose):
        self.measured_pose_pub.publish(pose)

    def publish_sent_pose(self, pose):
        self.sent_pose_pub.publish(pose)

    def get_max_queued(self):
        return self.max_queued
