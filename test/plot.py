#!/bin/python
import matplotlib.pyplot as plt
import csv
import sys, os
import numpy as np
from os.path import expanduser
import stats as stats

<<<<<<< HEAD
dir = expanduser("~/egm_control/data")
=======
home = expanduser("~")

def csv_to_arrays(csvname, t, x):
	with open(csvname,'r') as csvfile:
	    plots = csv.reader(csvfile, delimiter=',')
	    tref = float(next(plots)[0])
	    xref = float(next(plots)[1])
	    for row in plots:
	        t.append((float(row[0])-tref)/1.0e9)
	        x.append(float(row[1])-xref)

def import_test(csvname):
	t1 = []
	t2 = []
	x1 = []
	x2 = []
	csv_to_arrays(home + '/egm_control/data/measured_pose_' + csvname + '.txt', t1, x1)
	csv_to_arrays(home + '/egm_control/data/sent_pose_' + csvname + '.txt', t2, x2)
	index_tref = np.min(np.where(np.array(t1) > 0.5))
	t1 = t1[index_tref:]
	t1[:] = [t-0.5 for t in t1]
	x1 = x1[index_tref:]
	t2 = t2[index_tref:]
	t2[:] = [t-0.5 for t in t2]
	x2 = x2[index_tref:]
	return t1, t2, x1, x2

def get_last_file():
    dirFiles = os.listdir(home + '/egm_control/data')
    if 'plot.py' in dirFiles: dirFiles.remove('plot.py')
    if 'old' in dirFiles: dirFiles.remove('old')
    dirFiles.sort(key=lambda f: int(filter(str.isdigit, f)))
    return filter(str.isdigit, dirFiles[-1])
>>>>>>> 335f065583b2d711936075031bc2ec166160d516

if sys.argv[1] == 'last':
    m = stats.get_last_file(dir)
    print('Showing last measurement: ' + m)
    t1, t2, x1, x2 = stats.import_test(dir, m)
else:
    t1, t2, x1, x2 = stats.import_test(dir, sys.argv[1])
speed = float(sys.argv[2]) # mm/s

d1, d2, d3, d4 = stats.get_stats(t1, t2, x1, x2, speed)
print('Mid measurement delay, theoretical-sent (avg): ' + str(d1*1000) + ' ms')
print('Mid measurement delay, sent-measured (avg): ' + str(d2*1000) + ' ms')
print('Stabilization delay, sent-measured: ' + str(d3) + ' s')
print('Stabilization delay, theoretical-measured: ' + str(d4) + ' s')

ax = plt.subplot(111)
ax.plot(t1,x1, label='Position (measured), '+str(speed)+' mm/s')
ax.plot(t2,x2, label='Position (sent), '+str(speed)+' mm/s')
ax.plot(t1, [t*speed for t in t1], '--', label='Constant speed at '+str(speed)+' mm/s')
plt.xlabel('Time (s)')
plt.ylabel('Traveled distance (mm)')
chartBox = ax.get_position()
ax.set_position([chartBox.x0, chartBox.y0, chartBox.width*0.6, chartBox.height])
ax.legend(loc='upper center', bbox_to_anchor=(1.45, 0.8), shadow=True, ncol=1)
plt.show()
