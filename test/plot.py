#!/bin/python
import matplotlib.pyplot as plt
import csv
import sys, os
import numpy as np
from os.path import expanduser
import stats as stats

if sys.argv[3]:
    dir = expanduser("~/egm_control/data/" + sys.argv[3])
else:
    dir = expanduser("~/egm_control/data")

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
