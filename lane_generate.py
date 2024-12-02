from time import sleep
import pybullet as p
import pybullet_data
from models.goal import Goal
from models.road import Road
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as si
import os

#bus = p.loadURDF('./models/test.urdf',[10,0,3])
#number_of_joints = p.getNumJoints(bus)
# angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
# throttle = p.addUserDebugParameter('Throttle', 0, 20, 0)
#plane = p.loadURDF('./models/simpleplane.urdf')
#randomly select control points
control_1 = np.random.randint([20,-200],[200,0])
control_2 = np.random.randint([-200,-350],[200,-150])
#points = np.array([[0, 0], [160, -150], [120, -240], [0, -350]])
#generate b-spline curve according to selected control points
points = np.array([[0, 0], control_1, control_2, [0, -350]])
x = points[:,0]
y = points[:,1]
direction = np.random.choice([-1,1])
y = y*direction
t = range(len(x))
print(control_1)

ipl_t = np.linspace(0.0, len(points) - 1, 100)

x_tup = si.splrep(t, x, k=3)
y_tup = si.splrep(t, y, k=3)
x_i = si.splev(ipl_t, x_tup)
y_i = si.splev(ipl_t, y_tup)
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
plane = p.loadURDF('./models/simpleplane.urdf',basePosition=[0,0,0])
p.resetDebugVisualizerCamera(cameraDistance = 5,cameraYaw = -90,cameraPitch = -40, cameraTargetPosition = [-0.6,-0.1,-0.15])
pointsId = {}
road_l = {}
road_r = {}
k=0
p.resetDebugVisualizerCamera(cameraDistance = 150,cameraYaw = -90,cameraPitch = -89, cameraTargetPosition = [100,0,0])
for i in range(100):
    goal = (x_i[i],y_i[i])
    #goal = (y_i[i],-x_i[i])
    #print(goal)
    pointsId[i]=Goal(goal)
    #_ = Road(goal)
    # road_l[i],road_r[i]=Road(goal)

#print(pointsId)
car = p.loadURDF('./Truck-Trailer/truck_trailer/resources/TT/urdf/TT.urdf',
                 basePosition = [-10,2,3])
# car = p.loadURDF('./Truck-Trailer/truck_trailer/resources/bus.urdf',
#  
#                basePosition = [10,0,3])
#sleep(1000)
angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
throttle = p.addUserDebugParameter('Throttle', 0, 10, 0)

sleep(3)
wheel_indices = [1, 3, 4, 5]
hinge_indices = [0, 2]
#p.resetDebugVisualizerCamera(cameraDistance = 150,cameraYaw = -90,cameraPitch = -89, cameraTargetPosition = [100,0,0])
while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    for joint_index in wheel_indices:
        p.setJointMotorControl2(car, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttle)
    for joint_index in hinge_indices:
        p.setJointMotorControl2(car, joint_index,
                                p.POSITION_CONTROL, 
                                targetPosition=user_angle)
    p.stepSimulation()
