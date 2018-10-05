#!/bin/python
import csv
import sys, os
import numpy as np

def csv_to_arrays(csvname, t, x):
	with open(csvname,'r') as csvfile:
	    plots = csv.reader(csvfile, delimiter=',')
	    tref = float(next(plots)[0])
	    xref = float(next(plots)[1])
	    for row in plots:
	        t.append((float(row[0])-tref)/1.0e9)
	        x.append(float(row[1])-xref)

def import_test(dir, csvname):
	t1 = []
	t2 = []
	x1 = []
	x2 = []
	csv_to_arrays(dir + '/measured_pose_' + csvname + '.txt', t1, x1)
	csv_to_arrays(dir + '/sent_pose_' + csvname + '.txt', t2, x2)
	index_tref = np.min(np.where(np.array(t1) > 0.49))
	t1 = t1[index_tref:]
	t1[:] = [t-0.49 for t in t1]
	x1 = x1[index_tref:]
	t2 = t2[index_tref:]
	t2[:] = [t-0.49 for t in t2]
	x2 = x2[index_tref:]
	return t1, t2, x1, x2

def get_last_file(dir):
    dirFiles = os.listdir(dir)
    if 'plot.py' in dirFiles: dirFiles.remove('plot.py')
    if 'old' in dirFiles: dirFiles.remove('old')
    dirFiles.sort(key=lambda f: int(filter(str.isdigit, f)))
    return filter(str.isdigit, dirFiles[-1])

def get_stats(t1, t2, x1, x2, speed):
	# Mid-measurement delay between theoretical and sent
	d1 = []
	for i in range(int(0.3*len(t2)), int(0.7*len(t2))):
		xref = x2[i]
		d1.append(t2[i]-xref/speed)
	d1 = np.average(d1)

	# Mid-measurement delay between sent and measured
	d2 = []
	for i in range(int(0.3*len(t1)), int(0.7*len(t1))):
		xref = x1[i]
		d2.append(t1[i]-t2[np.min(np.where(np.array(x2) >= xref))])
	d2 = np.average(d2)

	# Stabilization delay at end
	xref = x2[-1]
	d3 = t1[np.min(np.where(abs(np.array(x1)-xref) < 0.05))]-t2[np.min(np.where(np.array(x2) == xref))]

	xref = x2[-1]
	d4 = t1[np.min(np.where(abs(np.array(x1)-xref) < 0.05))]-xref/speed
	return d1, d2, d3, d4
