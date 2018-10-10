#!/bin/python
import rosbag
import sys, os
import rospy
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag(sys.argv[1])
speed = 5.0
pre_time = 0.5
time = 10.5

start_times = []
sent_poses_times = []
sent_poses = []
prev_vel = 0.0

for topic, msg, t in bag.read_messages(topics='/command_pose'):
	if prev_vel == 0.0 and msg.pose.position.x == speed:
		start_times.append(t)
	prev_vel = msg.pose.position.x

for start_time in start_times:
	t0 = []
	x0 = []
	t1 = []
	x1 = []
	t2 = []
	x2 = []
	xref = 0.0

	for topic, msg, t in bag.read_messages(topics='/command_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		t0.append(t.to_sec()-start_time.to_sec())
		x0.append(msg.pose.position.x)

	for topic, msg, t in bag.read_messages(topics='/sent_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		if xref == 0.0:
			xref = msg.pose.position.x
		t1.append(t.to_sec()-start_time.to_sec())
		x1.append(msg.pose.position.x-xref)

	for topic, msg, t in bag.read_messages(topics='/measured_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		t2.append(t.to_sec()-start_time.to_sec())
		x2.append(msg.pose.position.x-xref)

	print((t1[np.min(np.where(np.array(x1) > x1[0]))]-t0[np.min(np.where(np.array(x0) > 0))])*1000)
	ax = plt.subplot(111)
	ax.plot(t0, x0, label='Expected position (theoretical), '+str(speed)+' mm/s')
	ax.plot(t1, x1, label='Position (sent), '+str(speed)+' mm/s')
	ax.plot(t2, x2, label='Position (measured), '+str(speed)+' mm/s')
	# ax.plot(t1, [t*speed for t in t1], '--', label='Constant speed at '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Traveled distance (mm)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()
