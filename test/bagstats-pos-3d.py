#!/bin/python
import rosbag
import sys, os
import rospy
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag(sys.argv[1])
plot = True
speed = 5.0
pre_time = 0.5
time = 1.0e5

for topic, msg, t in bag.read_messages(topics='/command_pose'):
	start_time = msg.header.stamp
	break

d1x = []
d2x = []
d3x = []
d1y = []
d2y = []
d3y = []
d1z = []
d2z = []
d3z = []

t0 = []
x0 = []
y0 = []
z0 = []
t1 = []
x1 = []
y1 = []
z1 = []
t2 = []
x2 = []
y2 = []
z2 = []
v1x = []
v1y = []
v1z = []
a1x = []
a1y = []
a1z = []
v2x = []
v2y = []
v2z = []
a2x = []
a2y = []
a2z = []

prev_x1_time = start_time.to_sec()
prev_y1_time = start_time.to_sec()
prev_z1_time = start_time.to_sec()
for topic, msg, t in bag.read_messages(topics='/sent_pose'):
	t1.append(msg.header.stamp.to_sec()-start_time.to_sec())
	x1.append(msg.pose.position.x)
	y1.append(msg.pose.position.y)
	z1.append(msg.pose.position.z)
	if len(x1) > 1:
		v1x.append((x1[-1]-x1[-2])/(msg.header.stamp.to_sec()-prev_x1_time))
		if len(v1x) > 1:
			a1x.append((v1x[-1]-v1x[-2])/(msg.header.stamp.to_sec()-prev_x1_time))
		prev_x1_time = msg.header.stamp.to_sec()
	if len(y1) > 1:
		v1y.append((y1[-1]-y1[-2])/(msg.header.stamp.to_sec()-prev_y1_time))
		if len(v1y) > 1:
			a1y.append((v1y[-1]-v1y[-2])/(msg.header.stamp.to_sec()-prev_y1_time))
		prev_y1_time = msg.header.stamp.to_sec()
	if len(z1) > 1:
		v1z.append((z1[-1]-z1[-2])/(msg.header.stamp.to_sec()-prev_z1_time))
		if len(v1z) > 1:
			a1z.append((v1z[-1]-v1z[-2])/(msg.header.stamp.to_sec()-prev_z1_time))
		prev_z1_time = msg.header.stamp.to_sec()

for topic, msg, t in bag.read_messages(topics='/command_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
	t0.append(msg.header.stamp.to_sec()-start_time.to_sec())
	x0.append(msg.pose.position.x)
	y0.append(msg.pose.position.y)
	z0.append(msg.pose.position.z)

prev_x2_time = start_time.to_sec()
prev_y2_time = start_time.to_sec()
prev_z2_time = start_time.to_sec()
for topic, msg, t in bag.read_messages(topics='/measured_pose', start_time=start_time-rospy.rostime.Duration.from_sec(pre_time), end_time=start_time+rospy.rostime.Duration.from_sec(time)):
	t2.append(msg.header.stamp.to_sec()-start_time.to_sec())
	x2.append(msg.pose.position.x)
	y2.append(msg.pose.position.y)
	z2.append(msg.pose.position.z)
	if len(x2) > 1:
		v2x.append((x2[-1]-x2[-2])/(msg.header.stamp.to_sec()-prev_x2_time))
		if len(v2x) > 1:
			a2x.append((v2x[-1]-v2x[-2])/(msg.header.stamp.to_sec()-prev_x2_time))
		prev_x2_time = msg.header.stamp.to_sec()
	if len(y2) > 1:
		v2y.append((y2[-1]-y2[-2])/(msg.header.stamp.to_sec()-prev_y2_time))
		if len(v2y) > 1:
			a2y.append((v2y[-1]-v2y[-2])/(msg.header.stamp.to_sec()-prev_y2_time))
		prev_y2_time = msg.header.stamp.to_sec()
	if len(z2) > 1:
		v2z.append((z2[-1]-z2[-2])/(msg.header.stamp.to_sec()-prev_z2_time))
		if len(v2z) > 1:
			a2z.append((v2z[-1]-v2z[-2])/(msg.header.stamp.to_sec()-prev_z2_time))
		prev_z2_time = msg.header.stamp.to_sec()

# d1x.append((t1[np.min(np.where(np.array(x1) > x1[0]))]-t0[np.min(np.where(np.array(x0) > 0))])*1000.0)
# d1y.append((t1[np.min(np.where(np.array(y1) > y1[0]))]-t0[np.min(np.where(np.array(y0) > 0))])*1000.0)
# # d1z.append((t1[np.min(np.where(np.array(z1) > z1[0]))]-t0[np.min(np.where(np.array(z0) > 0))])*1000.0)
#
# # start = np.min(np.where(np.array(t1) >= 1.0))
# # end = np.min(np.where(np.array(t1) >= time-2.0))
# # d2ind = []
# # for i in range(start, end):
# # 	d2ind.append(t2[np.min(np.where(np.array(x2) >= x1[i]))]-t1[i])
# # d2x.append(np.average(d2ind)*1000.0)
# # d2ind = []
# # for i in range(start, end):
# # 	d2ind.append(t2[np.min(np.where(np.array(y2) >= y1[i]))]-t1[i])
# # d2y.append(np.average(d2ind)*1000.0)
# # d2ind = []
# # for i in range(start, end):
# # 	d2ind.append(t2[np.min(np.where(np.array(z2) >= z1[i]))]-t1[i])
# # d2z.append(np.average(d2ind)*1000.0)
#
# d3x.append((t2[np.min(np.where(abs(np.array(x2)-x1[-1]) < 0.01))]-t1[np.min(np.where(np.array(x1) == x1[-1]))])*1000.0)
# d3y.append((t2[np.min(np.where(abs(np.array(y2)-y1[-1]) < 0.01))]-t1[np.min(np.where(np.array(y1) == y1[-1]))])*1000.0)
# # d3z.append((t2[np.min(np.where(abs(np.array(z2)-z1[-1]) < 0.01))]-t1[np.min(np.where(np.array(z1) == z1[-1]))])*1000.0)
#
# print("Movement start delay (delta t1)")
# print("avg for x: " + str(np.average(d1x)) + " ms")
# print("std for x: " + str(np.std(d1x)) + " ms")
# print("avg for y: " + str(np.average(d1y)) + " ms")
# print("std for y: " + str(np.std(d1y)) + " ms")
# # print("avg for z: " + str(np.average(d1z)) + " ms")
# # print("std for z: " + str(np.std(d1z)) + " ms")
#
# # print("Mid-measurement delay, sent-measured positions (delta t2)")
# # print("avg for x: " + str(np.average(d2x)) + " ms")
# # print("std for x: " + str(np.std(d2x)) + " ms")
# # print("avg for y: " + str(np.average(d2y)) + " ms")
# # print("std for y: " + str(np.std(d2y)) + " ms")
# # print("avg for z: " + str(np.average(d2z)) + " ms")
# # print("std for z: " + str(np.std(d2z)) + " ms")
#
# print("Stabilization delay, sent-measured positions (delta t3)")
# print("avg for x: " + str(np.average(d3x)) + " ms")
# print("std for x: " + str(np.std(d3x)) + " ms")
# print("avg for y: " + str(np.average(d3y)) + " ms")
# print("std for y: " + str(np.std(d3y)) + " ms")
# # print("avg for z: " + str(np.average(d3z)) + " ms")
# # print("std for z: " + str(np.std(d3z)) + " ms")

if plot:
	ax = plt.subplot(111)
	ax.plot(t2, x2, label='Position x (measured), '+str(speed)+' mm/s')
	ax.plot(t1, x1, label='Position x (sent), '+str(speed)+' mm/s')
	ax.plot(t0, x0, '--', label='Expected position (theoretical), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Traveled distance (mm)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2, y2, label='Position y (measured), '+str(speed)+' mm/s')
	ax.plot(t1, y1, label='Position y (sent), '+str(speed)+' mm/s')
	ax.plot(t0, y0, '--', label='Expected position y (theoretical), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Traveled distance (mm)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2, z2, label='Position z (measured), '+str(speed)+' mm/s')
	ax.plot(t1, z1, label='Position z (sent), '+str(speed)+' mm/s')
	ax.plot(t0, z0, '--', label='Expected position z (theoretical), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Traveled distance (mm)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2[1:], v2x, label='Velocity x (measured), '+str(speed)+' mm/s')
	ax.plot(t1[1:], v1x, label='Velocity x (sent), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Velocity (mm/s)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2[1:], v2y, label='Velocity y (measured), '+str(speed)+' mm/s')
	ax.plot(t1[1:], v1y, label='Velocity y (sent), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Velocity (mm/s)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2[1:], v2z, label='Velocity z (measured), '+str(speed)+' mm/s')
	ax.plot(t1[1:], v1z, label='Velocity z (sent), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Velocity (mm/s)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2[2:], a2x, label='Acceleration x (measured), '+str(speed)+' mm/s')
	ax.plot(t1[2:], a1x, label='Acceleration x (sent), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Acceleration (mm/s^2)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2[2:], a2y, label='Acceleration y (measured), '+str(speed)+' mm/s')
	ax.plot(t1[2:], a1y, label='Acceleration y (sent), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Acceleration (mm/s^2)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()

	ax = plt.subplot(111)
	ax.plot(t2[2:], a2z, label='Acceleration z (measured), '+str(speed)+' mm/s')
	ax.plot(t1[2:], a1z, label='Acceleration z (sent), '+str(speed)+' mm/s')
	plt.xlabel('Time (s)')
	plt.ylabel('Acceleration (mm/s^2)')
	chartBox = ax.get_position()
	ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
	ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
	plt.show()
