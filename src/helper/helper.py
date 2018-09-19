import rospy
import subprocess, sys, os
import csv
import pdb
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
import os
from visualization_msgs.msg import Marker
import geometry_msgs
import json, copy

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for term in list_output.split("\n"):
        if (term.startswith(s)):
            os.system("rosnode kill " + term)
            print "rosnode kill " + term

def publish_time(time, time_pub):
    time_msg = Float32()
    time_msg.data = time
    time_pub.publish(time_msg)

def csv2dict(filename):
    # input_file = csv.DictReader(open(filename))
    with open (filename) as f:
        data = f.read()

    reader = csv.DictReader(data.splitlines(0)[0:])
    lines = list(reader)
    # for row in reader:
    # counties = {k: v for (k,v in ((line['%time']) for line in lines)}
    name_list = lines[0].keys()
    d = {}
    for key in name_list:
        d[key] = []
        for data in range(len(lines)):
            d[key].append(float(lines[data][key]))
    return d

def trajectory_viz(x_vec, y_vec, z_vec, frame_id = "/track_start", line_thick = 0.005, color = (0.,0.,1., 1.), indices = None, marker_type=Marker.LINE_STRIP):
    line_strip = Marker()
    line_strip.header.frame_id  = frame_id
    line_strip.header.stamp = rospy.Time.now()
    line_strip.action =  Marker.ADD
    line_strip.pose.orientation.w =  1.0
    line_strip.type = marker_type
    line_strip.scale.x = line_thick
    # // Line strip is blue
    line_strip.color.r = color[0]
    line_strip.color.g = color[1]
    line_strip.color.b = color[2]
    line_strip.color.a = color[3]

    if indices==None:
        for i in range(0, len(x_vec), 15):
            # import pdb;pdb.set_trace()
            # print x_vec
            x= x_vec[i]
            y= y_vec[i]
            z= z_vec[i]
            # print x_vec[i], y_vec[i], z_vec[i],i

            p =geometry_msgs.msg.Point()
            p.x = x
            p.y = y
            p.z = z

            line_strip.points.append(p)
        return line_strip
    else:
        if np.abs(indices[1]-indices[0])>1000 or indices[0] in range(1700,2000) or indices[0] <0:
            # import pdb;pdb.set_trace()
            line_strip2 = copy.deepcopy(line_strip);
            print indices
            for i in range(100, 1750, 15):
                x= x_vec[i]
                y= y_vec[i]
                z= z_vec[i]

                p =geometry_msgs.msg.Point()
                p.x = x
                p.y = y
                p.z = z

                line_strip.points.append(p)
            for i in range(1985, 3630, 15):
                x= x_vec[i]
                y= y_vec[i]
                z= z_vec[i]

                p =geometry_msgs.msg.Point()
                p.x = x
                p.y = y
                p.z = z

                line_strip2.points.append(p)
        else:
            # import pdb;pdb.set_trace()
            line_strip2 = copy.deepcopy(line_strip);
            print indices
            for i in range(0, indices[0], 15):
                x= x_vec[i]
                y= y_vec[i]
                z= z_vec[i]

                p =geometry_msgs.msg.Point()
                p.x = x
                p.y = y
                p.z = z

                line_strip.points.append(p)

            for i in range(indices[1], len(x_vec), 15):
                x= x_vec[i]
                y= y_vec[i]
                z= z_vec[i]

                p =geometry_msgs.msg.Point()
                p.x = x
                p.y = y
                p.z = z

                line_strip2.points.append(p)
        return line_strip, line_strip2

def plot_trajectory(date):
    #Compile rosbag to .csv and plot trajectory
    val = os.system(".././bag2csv.sh %s" %'/home/mcube/pushing_benchmark_data/'  +date)
    base_path ='/home/mcube/pushing_benchmark_data/' + date + 'csv'
    sensed_cart = csv2dict(base_path + '/' + date + '__cart_sensed_states.csv')
    command_cart = csv2dict(base_path + '/' + date + '__cart_command_states.csv')
    object_pose = csv2dict(base_path + '/' + date + '__object_pose.csv')
    time = csv2dict(base_path + '/' + date + '__time.csv')
    time_x = np.array(time['%time']) - np.array(time['%time'][0])
    time_y = np.array(time['field.data'])
    time_sensed = (np.array(sensed_cart['%time']) - sensed_cart['%time'][0])/1e9
    x_sensed = object_pose['field.data0']
    y_sensed = object_pose['field.data1']
    time_command = (np.array(command_cart['%time']) - command_cart['%time'][0])/1e9
    x_command = command_cart['field.data0']
    y_command = command_cart['field.data1']
    # plt.plot(time_sensed, x_sensed);plt.plot(time_command, x_command);plt.show()
    # plt.plot(time_x, time_y);plt.show()
    plt.plot(x_sensed, y_sensed)
    plt.axis('equal')
    plt.show()

    pdb.set_trace()
