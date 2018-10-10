#!/bin/python
import csv
import sys, os
import numpy as np
from os.path import expanduser
import stats as stats

dir = expanduser("~/egm_control/data/" + sys.argv[1])

dirFiles = os.listdir(dir)
if 'plot.py' in dirFiles: dirFiles.remove('plot.py')
if 'old' in dirFiles: dirFiles.remove('old')
dirFiles.sort(key=lambda f: int(filter(str.isdigit, f)))

speed = float(sys.argv[2]) # mm/s

d1s = []
d2s = []
d3s = []
d4s = []

for m in dirFiles:
	# Happens twice per number!
	t1, t2, x1, x2 = stats.import_test(dir, filter(str.isdigit, m))
	d1, d2, d3, d4 = stats.get_stats(t1, t2, x1, x2, speed)
	d1s.append(d1)
	d2s.append(d2)
	d3s.append(d3)
	d4s.append(d4)

d1 = np.min(d1s)
print('Mid measurement delay, theoretical-sent (min): ' + str(d1*1000) + ' ms')
d1 = np.average(d1s)
print('Mid measurement delay, theoretical-sent (avg): ' + str(d1*1000) + ' ms')
d1 = np.max(d1s)
print('Mid measurement delay, theoretical-sent (max): ' + str(d1*1000) + ' ms')
d1 = np.std(d1s)
print('Mid measurement delay, theoretical-sent (stdev): ' + str(d1*1000) + ' ms')

d2 = np.min(d2s)
print('Mid measurement delay, sent-measured (min): ' + str(d2*1000) + ' ms')
d2 = np.average(d2s)
print('Mid measurement delay, sent-measured (avg): ' + str(d2*1000) + ' ms')
d2 = np.max(d2s)
print('Mid measurement delay, sent-measured (max): ' + str(d2*1000) + ' ms')
d2 = np.std(d2s)
print('Mid measurement delay, sent-measured (stdev): ' + str(d2*1000) + ' ms')

d3 = np.min(d3s)
print('Stabilization delay, sent-measured (min): ' + str(d3) + ' s')
d3 = np.average(d3s)
print('Stabilization delay, sent-measured (avg): ' + str(d3) + ' s')
d3 = np.max(d3s)
print('Stabilization delay, sent-measured (max): ' + str(d3) + ' s')
d3 = np.std(d3s)
print('Stabilization delay, sent-measured (stdev): ' + str(d3) + ' s')

d4 = np.min(d4s)
print('Stabilization delay, theoretical-measured (min): ' + str(d4) + ' s')
d4 = np.average(d4s)
print('Stabilization delay, theoretical-measured (avg): ' + str(d4) + ' s')
d4 = np.max(d4s)
print('Stabilization delay, theoretical-measured (max): ' + str(d4) + ' s')
d4 = np.std(d4s)
print('Stabilization delay, theoretical-measured (stdev): ' + str(d4) + ' s')
