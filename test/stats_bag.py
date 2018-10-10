#!/bin/python
import rosbag
import sys, os
import rospy
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag(sys.argv[1])
plot = False
speed = 5.0
pre_time = 0.5
time = 10.5

start_times = []
prev_vel = 0.0

for topic, msg, t in bag.read_messages(topics='/command_pose'):
	if prev_vel == 0.0 and msg.pose.position.x == speed:
		start_times.append(msg.header.stamp)
	prev_vel = msg.pose.position.x

d1 = []
d2 = []
d3 = []

for start_time in start_times:
	t0 = []
	x0 = []
	t1 = []
	x1 = []
	t2 = []
	x2 = []
	v1 = []
	a1 = []
	v2 = []
	a2 = []
	xref = 0.0

	prev_x1_time = start_time.to_sec()
	for topic, msg, t in bag.read_messages(topics='/sent_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		if xref == 0.0:
			xref = msg.pose.position.x
		t1.append(msg.header.stamp.to_sec()-start_time.to_sec())
		x1.append(msg.pose.position.x-xref)
		if len(x1) > 1:
			v1.append((x1[-1]-x1[-2])/(msg.header.stamp.to_sec()-prev_x1_time))
			if len(v1) > 1:
				a1.append((v1[-1]-v1[-2])/(msg.header.stamp.to_sec()-prev_x1_time))
			prev_x1_time = msg.header.stamp.to_sec()

	lastx = 0.0
	prev_x0_time = start_time.to_sec()
	for topic, msg, t in bag.read_messages(topics='/command_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		t0.append(msg.header.stamp.to_sec()-start_time.to_sec())
		lastx += msg.pose.position.x*(msg.header.stamp.to_sec()-prev_x0_time)
		x0.append(lastx)
		prev_x0_time = msg.header.stamp.to_sec()

	prev_x2_time = start_time.to_sec()
	for topic, msg, t in bag.read_messages(topics='/measured_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		t2.append(msg.header.stamp.to_sec()-start_time.to_sec())
		x2.append(msg.pose.position.x-xref)
		if len(x2) > 1:
			v2.append((x2[-1]-x2[-2])/(msg.header.stamp.to_sec()-prev_x2_time))
			if len(v2) > 1:
				a2.append((v2[-1]-v2[-2])/(msg.header.stamp.to_sec()-prev_x2_time))
			prev_x2_time = msg.header.stamp.to_sec()

	d1.append((t1[np.min(np.where(np.array(x1) > x1[0]))]-t0[np.min(np.where(np.array(x0) > 0))])*1000)

	if plot:
		ax = plt.subplot(111)
		ax.plot(t2, x2, label='Position (measured), '+str(speed)+' mm/s')
		ax.plot(t1, x1, label='Position (sent), '+str(speed)+' mm/s')
		ax.plot(t0, x0, '--', label='Expected position (theoretical), '+str(speed)+' mm/s')
		plt.xlabel('Time (s)')
		plt.ylabel('Traveled distance (mm)')
		chartBox = ax.get_position()
		ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
		ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
		plt.show()

		ax = plt.subplot(111)
		ax.plot(t2[1:], v2, label='Velocity (measured), '+str(speed)+' mm/s')
		ax.plot(t1[1:], v1, label='Velocity (sent), '+str(speed)+' mm/s')
		plt.xlabel('Time (s)')
		plt.ylabel('Velocity (mm/s)')
		chartBox = ax.get_position()
		ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
		ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
		plt.show()

		ax = plt.subplot(111)
		ax.plot(t2[2:], a2, label='Acceleration (measured), '+str(speed)+' mm/s')
		ax.plot(t1[2:], a1, label='Acceleration (sent), '+str(speed)+' mm/s')
		plt.xlabel('Time (s)')
		plt.ylabel('Acceleration (mm/s^2)')
		chartBox = ax.get_position()
		ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
		ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
		plt.show()
