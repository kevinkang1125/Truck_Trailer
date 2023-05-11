from time import sleep
import pybullet as p
import pybullet_data
from models.goal import Goal
from models.road import Road
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si
import os
import math

points = np.array([[0, 0], [105, 60], [150, -60], [270, 90]])
x = points[:,0]
y = points[:,1]

t = range(len(x))

ipl_t = np.linspace(0.0, len(points) - 1, 100)

x_tup = si.splrep(t, x, k=3)
y_tup = si.splrep(t, y, k=3)
x_i = si.splev(ipl_t, x_tup)
y_i = si.splev(ipl_t, y_tup)
#p.resetDebugVisualizerCamera(cameraDistance = 5,cameraYaw = -90,cameraPitch = -40, cameraTargetPosition = [-0.6,-0.1,-0.15])
pointsId = {}
road_l = {}
road_r = {}
goallist = []
interval_list = []
k=0
dist = 0
for i in range(100):
    if i >=1:
        goal = (x_i[i],y_i[i])
        goallist.append(goal)
        #pointsId[i]=Goal(self.client,goal)
        interval = math.sqrt((x_i[i-1]-x_i[i])**2+(y_i[i-1]-y_i[i])**2)
        interval_list.append(interval)
        print(interval)
        dist += interval
mean = sum(interval_list)/len(interval_list)   
print(dist)
print(min(interval_list),max(interval_list),mean)
        

