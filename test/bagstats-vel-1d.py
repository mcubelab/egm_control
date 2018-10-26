#!/bin/python
import rosbag
import sys, os
import rospy
import numpy as np
import matplotlib.pyplot as plt

plot = True
stats = True
vicon = False

bag = rosbag.Bag(sys.argv[1])
speed = 0.005
pre_time = 0.5
time = 11.5

start_times = []
prev_vel = 0.0

for topic, msg, t in bag.read_messages(topics='/command_pose'):
	if prev_vel == 0.0 and msg.pose.position.x == speed:
		start_times.append(msg.header.stamp)
	prev_vel = msg.pose.position.x

print("Number of measurements: " + str(len(start_times)))

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
	t3 = []
	x3 = []
	v1 = []
	a1 = []
	v2 = []
	a2 = []
	xref = 0.0

	prev_x2_time = start_time.to_sec()
	for topic, msg, t in bag.read_messages(topics='/measured_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		if xref == 0.0:
			xref = msg.pose.position.x
		t2.append(msg.header.stamp.to_sec()-start_time.to_sec())
		x2.append(msg.pose.position.x-xref)
		if len(x2) > 1:
			v2.append((x2[-1]-x2[-2])/(msg.header.stamp.to_sec()-prev_x2_time))
			if len(v2) > 1:
				a2.append((v2[-1]-v2[-2])/(msg.header.stamp.to_sec()-prev_x2_time))
			prev_x2_time = msg.header.stamp.to_sec()

	prev_x1_time = start_time.to_sec()
	lastx = 0.0
	for topic, msg, t in bag.read_messages(topics='/sent_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
		t1.append(msg.header.stamp.to_sec()-start_time.to_sec())
		lastx += msg.pose.position.x*(msg.header.stamp.to_sec()-prev_x1_time)
		x1.append(lastx)
		# x1.append(msg.pose.position.x-xref)
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

	if vicon:
		x3ref = 0.0
		for topic, msg, t in bag.read_messages(topics='/vicon/CalibViconPlate/CalibViconPlate', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
			if x3ref == 0.0:
				x3ref = msg.transform.translation.x
			t3.append(msg.header.stamp.to_sec()-start_time.to_sec())
			x3.append(msg.transform.translation.x-x3ref)

	if stats:
		d1.append((t1[np.min(np.where(np.array(x1) > x1[0]))]-t0[np.min(np.where(np.array(x0) > 0))])*1000.0)
		# print((t1[np.min(np.where(np.array(x1) > x1[0]))]-t0[np.min(np.where(np.array(x0) > 0))])*1000)

		d2ind = []
		start = np.min(np.where(np.array(t1) >= 1.0))
		end = np.min(np.where(np.array(t1) >= time-2.0))
		for i in range(start, end):
			d2ind.append(t2[np.min(np.where(np.array(x2) >= x1[i]))]-t1[i])
		d2.append(np.average(d2ind)*1000.0)

		d3.append((t2[np.min(np.where(abs(np.array(x2)-x1[-1]) < 0.01/1000.0))]-t1[np.min(np.where(np.array(x1) == x1[-1]))])*1000.0)

	if plot:
		ax = plt.subplot(111)
		ax.plot(t2, x2, label='Position (measured), '+str(speed)+' m/s')
		ax.plot(t1, x1, label='Position (sent), '+str(speed)+' m/s')
		ax.plot(t0, x0, '--', label='Expected position (theoretical), '+str(speed)+' m/s')
		if vicon:
			ax.plot(t3, x3, label='Position (vicon), '+str(speed)+' m/s')
		plt.xlabel('Time (s)')
		plt.ylabel('Traveled distance (m)')
		chartBox = ax.get_position()
		ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
		ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
		plt.show()

		ax = plt.subplot(111)
		ax.plot(t2[1:], v2, label='Velocity (measured), '+str(speed)+' m/s')
		ax.plot(t1[1:], v1, label='Velocity (sent), '+str(speed)+' m/s')
		plt.xlabel('Time (s)')
		plt.ylabel('Velocity (m/s)')
		chartBox = ax.get_position()
		ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
		ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
		plt.show()

		ax = plt.subplot(111)
		ax.plot(t2[2:], a2, label='Acceleration (measured), '+str(speed)+' m/s')
		ax.plot(t1[2:], a1, label='Acceleration (sent), '+str(speed)+' m/s')
		plt.xlabel('Time (s)')
		plt.ylabel('Acceleration (m/s^2)')
		chartBox = ax.get_position()
		ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
		ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
		plt.show()

if stats:
	print("Movement start delay (delta t1)")
	print("min: " + str(np.min(d1)) + " ms")
	print("avg: " + str(np.average(d1)) + " ms")
	print("max: " + str(np.max(d1)) + " ms")
	print("std: " + str(np.std(d1)) + " ms")

	print("Mid-measurement delay, sent-measured positions (delta t2)")
	print("min: " + str(np.min(d2)) + " ms")
	print("avg: " + str(np.average(d2)) + " ms")
	print("max: " + str(np.max(d2)) + " ms")
	print("std: " + str(np.std(d2)) + " ms")

	print("Stabilization delay, sent-measured positions (delta t3)")
	print("min: " + str(np.min(d3)) + " ms")
	print("avg: " + str(np.average(d3)) + " ms")
	print("max: " + str(np.max(d3)) + " ms")
	print("std: " + str(np.std(d3)) + " ms")
