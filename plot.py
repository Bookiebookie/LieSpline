#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt


data_path = sys.argv[1]

gen_data = np.loadtxt(data_path, delimiter=',')
data = np.loadtxt('data/dataset-calib-imu1_512_16/mav0/imu0/data.csv', delimiter=',')


plt.figure()
plt.title('Gyro')
plt.plot(data[:,0], data[:, 1], 'r')
plt.plot(data[:,0], data[:, 2], 'g')
plt.plot(data[:,0], data[:, 3], 'b')

plt.plot(gen_data[:,0], gen_data[:, 1], 'm')
plt.plot(gen_data[:,0], gen_data[:, 2], 'y')
plt.plot(gen_data[:,0], gen_data[:, 3], 'c')


plt.figure()
plt.title('Accel')
plt.plot(data[:,0], data[:, 4], 'r')
plt.plot(data[:,0], data[:, 5], 'g')
plt.plot(data[:,0], data[:, 6], 'b')

plt.plot(gen_data[:,0], gen_data[:, 4], 'm')
plt.plot(gen_data[:,0], gen_data[:, 5], 'y')
plt.plot(gen_data[:,0], gen_data[:, 6], 'c')

plt.show()