import pybullet as p
import time
import pybullet_data
from time import sleep
from lane import Lane
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("./Truck-Trailer/truck_trailer/resources/simpleplane.urdf")
startPos = [-15,-2,3]
startOrientation = p.getQuaternionFromEuler([0,0,1.57])#65
boxId = p.loadURDF("./Truck-Trailer/truck_trailer/resources/TT/urdf/TT.urdf",startPos,startOrientation)
Goallist,PointsId,planeId,Track_dist = Lane(physicsClient).generate_lane()
sleep(3)
angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
throttle = p.addUserDebugParameter('Throttle', 0, 20, 0)
#hintch = p.addUserDebugParameter('hintch', -0.5, 0.5, 0)
hinge_indices = [0, 2]
wheel_indices = [1,3,4,5,]
#hintch_indices = []

# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
#p.disconnect()
while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    # user_hintch = p.readUserDebugParameter(hintch)
    for joint_index in wheel_indices:
        p.setJointMotorControl2(boxId, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttle)
    for joint_index in hinge_indices:
        p.setJointMotorControl2(boxId, joint_index,
                                p.POSITION_CONTROL, 
                                targetPosition=user_angle)
    # p.setJointMotorControl2(boxId, 6,
    #                             p.POSITION_CONTROL, 
    #                             targetPosition=user_hintch)
    hitch_value,_,_,_ = p.getJointState(boxId,6)
    #print(hitch_value)
    p.stepSimulation()
