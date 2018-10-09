#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def callback(pose, fs):
    fs[0].write("%d,%.4f,%.4f,%.4f\n" % (rospy.Time.now().to_sec()*1e9, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    fs[1].write("%d,%.4f,%.4f,%.4f\n" % (pose.header.stamp.secs*1e9+pose.header.stamp.nsecs, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))

if __name__ == '__main__':
    f1a = open("/home/mcube/egm_control/data/py-ros/measured_pose_" + str(int(time.time())) + ".txt", "w")
    f1b = open("/home/mcube/egm_control/data/py-header/measured_pose_" + str(int(time.time())) + ".txt", "w")
    f2a = open("/home/mcube/egm_control/data/py-ros/sent_pose_" + str(int(time.time())) + ".txt", "w")
    f2b = open("/home/mcube/egm_control/data/py-header/sent_pose_" + str(int(time.time())) + ".txt", "w")
    rospy.init_node('datasaver', anonymous=True)
    measured_pose_sub = rospy.Subscriber("/measured_pose", PoseStamped, callback, (f1a, f1b))
    sent_pose_sub = rospy.Subscriber("/sent_pose", PoseStamped, callback, (f2a, f2b))

    rospy.spin()
    f1a.close()
    f1b.close()
    f2a.close()
    f2b.close()
