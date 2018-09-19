#!/usr/bin/env python
import egm_pb2, egm_helper, helper
from time import sleep
import rospy
import math
import numpy as np
import time, datetime
# from std_msgs.msg import Empty
from std_srvs.srv import Empty
from push_control.srv import rosbag
import subprocess, sys, os

def initialize_rosbag(req):
    #Saving rosbag options
    name_of_bag  = str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")) + req.input
    # topics = ["/viewer/image_raw"]
    topics = ["/time", "/xc", "/uc", "/us", "/q_pusher_sensed", "/q_pusher_commanded", "/viewer/image_raw", "/viewer/image_raw/compressed", "/viewer/camera_info", "/frame/track_start_frame_pub", "/frame/vicon_world_frame_pub", "/frame/viewer_frame_pub"]
    #topics = ["/viewer/image_raw"]
    #topics = []
    dir_save_bagfile = os.environ['HOME'] + '/pushing_benchmark_data/'
    rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
    # return MODE_SRVResponse(ms.index_convertor(ms.predict(req.delta_x)))
    return []

def terminate_rosbag(req):
    helper.terminate_ros_node('/record')
    return []

    # return MODE_SRVResponse(ms.index_convertor(ms.predict(req.delta_x)))

if __name__=='__main__':
    rospy.init_node('save_rosbag')
    s1 = rospy.Service('start_rosbag', rosbag, initialize_rosbag)
    s2 = rospy.Service('stop_rosbag', rosbag, terminate_rosbag)
    rospy.spin()
