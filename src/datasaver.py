#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


def callback(pose, f):
    f.write("%d,%.4f,%.4f,%.4f\n" % (rospy.Time.now().to_sec()*1e9, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    # f.write("%d,%.4f,%.4f,%.4f\n" % (pose.header.stamp.secs*1e9+pose.header.stamp.nsecs, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))

if __name__ == '__main__':
    f1 = open("/home/mcube/egm_control/data/measured_pose_" + str(int(time.time())) + ".txt", "w")
    f2 = open("/home/mcube/egm_control/data/sent_pose_" + str(int(time.time())) + ".txt", "w")
    rospy.init_node('datasaver', anonymous=True)
    measured_pose_sub = rospy.Subscriber("/measured_pose", PoseStamped, callback, f1)
    sent_pose_sub = rospy.Subscriber("/sent_pose", PoseStamped, callback, f2)

    rospy.spin()
    f1.close()
    f2.close()
