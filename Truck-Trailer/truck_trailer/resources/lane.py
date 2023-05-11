from fileinput import filename
from http import client
from typing_extensions import Self
import pybullet as p
import os
import math
import scipy.interpolate as si
import numpy as np
from truck_trailer.resources.goal import Goal

class Lane:
    def __init__(self,client):
        self.client = client
        
        

    def generate_lane(self):
        control_1 = np.random.randint([100,-200],[200,-50])
        control_2 = np.random.randint([-200,-300],[200,-150])
        #generate b-spline curve according to selected control points
        points = np.array([[0, 0], control_1, control_2, [0, -350]])
        x = points[:,0]
        y = points[:,1] 
        direction = np.random.choice([-1,1])
        y = y*direction
        t = range(len(x))
        ipl_t = np.linspace(0.0, len(points) - 1, 100)

        x_tup = si.splrep(t, x, k=3)
        y_tup = si.splrep(t, y, k=3)
        x_i = si.splev(ipl_t, x_tup)
        y_i = si.splev(ipl_t, y_tup)
        planeId = p.loadURDF("./Truck-Trailer/truck_trailer/resources/simpleplane.urdf",
                            basePosition = [0,0,0],
                            physicsClientId = self.client)
        pointsId = {}
        goallist=[]
        dist = 0
        for i in range(100):
            goal = (x_i[i],y_i[i])
            goallist.append(goal)
            pointsId[i]=Goal(self.client,goal)
            interval = math.sqrt((x_i[i-1]-x_i[i])**2+(y_i[i-1]-y_i[i])**2)
            dist += interval
        #print(goallist)  
        return goallist,pointsId,planeId,dist