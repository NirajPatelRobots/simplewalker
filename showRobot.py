# -*- coding: utf-8 -*-
"""
show robot: visualize robot state information
Created  Mar 2022

@author: Niraj
"""
import numpy as np
import matplotlib.pyplot as plt
import csv

t = np.empty((0))
pos = np.empty((3,0))
euler = np.empty((3,0))
vel = np.empty((3,0))
angvel = np.empty((3,0))

with open("state_estimation/localizeTestLog.csv", 'r') as inFile:
    reader = csv.DictReader(inFile)
    for row in reader:
        t = np.append(t, row['time'])
        pos = np.hstack((pos, np.array(row['pos[3]'].split(), dtype=float).reshape((3,1))))
        euler = np.hstack((euler, np.array(row['euler[3]'].split(), dtype=float).reshape((3,1))))
        vel = np.hstack((vel, np.array(row['vel[3]'].split(), dtype=float).reshape((3,1))))
        angvel = np.hstack((angvel, np.array(row['angvel[3]'].split(), dtype=float).reshape((3,1))))

print("Read", pos.shape[1], "data points.")


#Plot position
plt.figure(1)
plt.clf()
ax = plt.subplot(321)
plt.suptitle("State position")
ax.clear()
ax.relim()
ax.autoscale()
plt.ylabel("x")
plt.plot(t, pos[0,:])
#plt.grid()
ax = plt.subplot(323)
plt.ylabel("y")
plt.plot(t, pos[1,:])
#plt.grid()
ax = plt.subplot(325)
plt.ylabel("z")
plt.plot(t, pos[2,:])
#plt.grid()
ax = plt.subplot(322)
plt.ylabel(r'$\alpha$')
plt.plot(t, euler[0,:])
#plt.grid()
ax = plt.subplot(324)
plt.ylabel(r'$\beta$')
plt.plot(t, euler[1,:])
#plt.grid()
ax = plt.subplot(326)
plt.ylabel(r'$\gamma$')
plt.plot(t, euler[2,:])
#plt.grid()



plt.show()

