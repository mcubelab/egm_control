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
        self.measured_pose_pub = rospy.Publisher("/measured_pose", PoseStamped, queue_size = 2)
        self.command_pose = None

    def load_command_pose(self, data):
        self.command_pose = data

    def get_command_pose(self):
        return self.command_pose

    def publish_joint_state(self, joints):
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [j for j in joints]
        js.velocity = [0.0 for i in xrange(6)]
        js.effort = [0.0 for i in xrange(6)]
        self.joint_state_pub.publish(js)

    def publish_measured_pose(self, pos, quaternion):
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = pos.x/1000
        pose.pose.position.y = pos.y/1000
        pose.pose.position.z = pos.z/1000
        pose.pose.orientation.x = quaternion.u1
        pose.pose.orientation.y = quaternion.u2
        pose.pose.orientation.z = quaternion.u3
        pose.pose.orientation.w = quaternion.u0
        self.measured_pose_pub.publish(pose)
