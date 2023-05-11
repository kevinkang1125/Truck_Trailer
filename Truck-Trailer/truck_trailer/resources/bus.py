from fileinput import filename
from typing_extensions import Self
import pybullet as p
import os
import math

class Bus:
    def __init__(self,client):
        self.client = client
        #f_name = os.path.join(os.path.dirname(__file__),'bus.urdf')
        self.car = p.loadURDF("./Truck-Trailer/truck_trailer/resources/bus.urdf",
                              basePosition = [-15,-2,3],
                              physicsClientId = client)
        self.steering_joints = [0,2]
        self.drive_joints = [1,3,4,5]
    
    def get_ids(self):
        
        return self.car,self.client
     
    def control(self,action):
        throttle,steering_angle = action
        throttle = 10*(throttle)+10
        steering_angle=0.55*steering_angle
        throttle = min(max(throttle, 0), 20)
        steering_angle = max(min(steering_angle,0.5),-0.5)
        # set steer control
        p.setJointMotorControlArray(self.car, self.steering_joints,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[steering_angle] * 2,
                                    physicsClientId=self.client)
        # set drive control
        self.joint_speed = throttle
        p.setJointMotorControlArray(
            bodyUniqueId=self.car,
            jointIndices=self.drive_joints,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[self.joint_speed] * 4,
            physicsClientId=self.client)
        
    def get_observation(self):
    # Get the position and orientation of the car in the simulation
        pos, ang = p.getBasePositionAndOrientation(self.car, self.client)
        ang = p.getEulerFromQuaternion(ang)
        ori = (math.cos(ang[2]), math.sin(ang[2]))
        pos = pos[:2]
        # Get the velocity of the car
        lin_vel,ang_vel= p.getBaseVelocity(self.car, self.client)
        vel = lin_vel[0:2]+(ang_vel[2],)
        # Concatenate position, orientation, velocity
        #observation = (pos + ori + vel)
        observation = (ori + vel)
        return observation