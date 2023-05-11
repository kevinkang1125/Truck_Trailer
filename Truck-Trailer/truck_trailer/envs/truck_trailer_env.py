from cmath import inf
from distutils.log import error
from turtle import done
import gym
import numpy as np
from pandas import array
import pybullet as p
import math
#from truck_trailer.resources.bus import Bus
from truck_trailer.resources.track import track
from truck_trailer.resources.goal import Goal
from truck_trailer.resources.lane import Lane
from truck_trailer.resources.TruckTrailer import Truck
import matplotlib.pyplot as plt
from gym.utils import seeding
from gym import spaces,error,utils

class TruckTrailerEnv(gym.Env):
    metadata = {'render.modes':['human']}
    def __init__(self,render : bool = False):
        self.action_space = spaces.Box(
            low = np.array([-0.99,-1],dtype= np.float32),
            high= np.array([1,1],dtype= np.float32)
        )
        self.observation_space = spaces.Box(
            low = np.array([-1,-1,0,0,-inf,-inf,0,-inf,-inf,0,-inf,-inf],dtype=np.float32),
            high = np.array([1,1,25,25,inf,inf,inf,inf,inf,inf,inf,inf],dtype=np.float32)
        ) 
        self.np_random,_ = seeding.np_random()
        self.viz = render
        self.client = p.connect(p.GUI if self.viz else p.DIRECT)
        
        p.setTimeStep = (1/10,self.client)
        
        self.car = None
        self.plane = None
        self.carId= None
        self.planeId = None
        self.target = None
        self.targetId = None
        self.Goallist = None
        self.PointsId = None
        self.done = False
        self.state = None
        self.dist_to_target = None
        self.target_range = 2
        self.Goal_near = None
        self.Goal_far = None
        self.Goal_pre = None
        self.Goal_near_rear = None
        self.Goal_far_rear = None
        self.Track_dist = None
        self.dist_to_near_rear = None
        self.k = 10
        self.v = 0.005
        self.w = 1
        self.timesteps = 0
        self.reach = 0
        self.reach_trailer = 0
        self.reset()

    def step(self, action):
        _dist_to_near = self.dist_to_near
        _dist_to_far = self.dist_to_far
        self.car.control(action)
        p.stepSimulation(physicsClientId = self.client)
        self.timesteps +=1
        car_ob = self.car.get_observation()
        #print(car_ob)
        # compute env react parameters
        car_pos = car_ob[:2]
        tire_1_pos = p.getLinkState(self.carId,4,self.client)[0]
        tire_2_pos = p.getLinkState(self.carId,5,self.client)[0]
        tire_3_pos = p.getLinkState(self.carId,7,self.client)[0]
        tire_4_pos = p.getLinkState(self.carId,8,self.client)[0]
        ang = p.getLinkState(self.carId,6,self.client)[1]
        ang = p.getEulerFromQuaternion(ang)
        trailer_ori = (math.cos(ang[2]-1.57), math.sin(ang[2]-1.57))
        tractor_x = (tire_1_pos[0]+tire_2_pos[0])/2
        tractor_y = (tire_1_pos[1]+tire_2_pos[1])/2
        trailer_x = (tire_3_pos[0]+tire_4_pos[0])/2
        trailer_y = (tire_3_pos[1]+tire_4_pos[1])/2
        car_center = (tractor_x,tractor_y)
        trailer_center = (trailer_x,trailer_y)
        #ego tractor coordinate transition
        local_near_x = (self.Goal_near[0]-car_center[0])*car_pos[0]+(self.Goal_near[1]-car_center[1])*car_pos[1]
        local_near_y = -(self.Goal_near[0]-car_center[0])*car_pos[1]+(self.Goal_near[1]-car_center[1])*car_pos[0]
        local_near = (local_near_x,local_near_y)
        local_far_x = (self.Goal_far[0]-car_center[0])*car_pos[1]+(self.Goal_far[1]-car_center[1])*car_pos[0]
        local_far_y = -(self.Goal_far[0]-car_center[0])*car_pos[0]+(self.Goal_far[1]-car_center[1])*car_pos[1]
        local_far = (local_far_x,local_far_y)
        #trailer coordinates
        local_rear_near_x = (self.Goal_near_rear[0]-trailer_center[0])*trailer_ori[0]+(self.Goal_near_rear[1]-trailer_center[1])*trailer_ori[1]
        local_rear_near_y = -(self.Goal_near_rear[0]-trailer_center[0])*trailer_ori[1]+(self.Goal_near_rear[1]-trailer_center[1])*trailer_ori[0]
        local_rear_near= (local_rear_near_x,local_rear_near_y)
        local_rear_far_x = (self.Goal_far_rear[0]-trailer_center[0])*trailer_ori[0]+(self.Goal_far_rear[1]-trailer_center[1])*trailer_ori[1]
        local_rear_far_y = -(self.Goal_far_rear[0]-trailer_center[0])*trailer_ori[1]+(self.Goal_far_rear[1]-trailer_center[1])*trailer_ori[0]
        local_rear_far= (local_rear_far_x,local_rear_far_y)
        self.dist_to_near = math.sqrt(((local_near_x)**2+(local_near_y)**2))
        self.dist_to_far = math.sqrt(((local_far_x)**2+(local_far_y)**2))
        self.dist_to_near_rear = math.sqrt(((local_rear_near_x)**2+(local_rear_near_y)**2))       
        observation = car_ob+(self.dist_to_near,)+local_near+(self.dist_to_far,)+local_far
        observation = np.array(observation,dtype=np.float32)
        #print(local_rear_near,local_rear_far)
        #deviation calculation for tractor
        top_tractor = abs((self.Goal_near[0]-self.Goal_far[0])*(self.Goal_near[1]-car_center[1])-(self.Goal_near[1]-self.Goal_far[1])*(self.Goal_near[0]-car_center[0]))
        bot_tractor = math.sqrt((self.Goal_near[0]-self.Goal_far[0])**2+(self.Goal_near[1]-self.Goal_far[1])**2)
        deviation_tractor = top_tractor/bot_tractor
        #deviation calculation for trailer
        top_trailer = abs((self.Goal_near_rear[0]-self.Goal_far_rear[0])*(self.Goal_near_rear[1]-trailer_center[1])-(self.Goal_near_rear[1]-self.Goal_far_rear[1])*(self.Goal_near_rear[0]-trailer_center[0]))
        bot_trailer = math.sqrt((self.Goal_near_rear[0]-self.Goal_far_rear[0])**2+(self.Goal_near_rear[1]-self.Goal_far_rear[1])**2)
        deviation_trailer = top_trailer/bot_trailer
        # Termination condition
        
        if self.reach <=2:
            done = bool(self.reach >= 98 or self.timesteps>=25000 or max(deviation_tractor,deviation_trailer)>=25)
        else:              
            done = bool(max(deviation_tractor,deviation_trailer)>=10 or self.reach >= 98 or self.timesteps>=25000 or self.reach_trailer >= 98)
        # constraint debug
        # if max(deviation_tractor,deviation_trailer)>=10 and self.reach > 2:
        #     print("off track",deviation_tractor,deviation_trailer)
        # if  self.reach >= 98:
        #     print("finish")          
        # Compute reward        
        reward_reach = 0.0
        reward_pro = 0.0
        reward_timeout = 0
        reward_out = 0
        reward_dev = 0
        
        
        if max(deviation_tractor,deviation_trailer)>=10 and self.reach >=3:
            reward_out = -200
            print("offroad_3")
        if max(deviation_tractor,deviation_trailer)>=25:
            reward_out = -200
            print("ofroad_2")
        if self.dist_to_near < self.target_range:
            reward_reach = 40
        if not done:
            reward_pro = (_dist_to_near-self.dist_to_near)*self.k 
            #reward_dev = (8-(deviation_tractor**2 + self.w * (deviation_trailer**2)))*self.v
            if local_near_x < 0 or self.dist_to_near<self.target_range:
            #if self.dist_to_near<self.target_range:
                self.reach += 1
                #print(local_near_x) 
                self.Goal_near = self.Goallist[self.reach]
                self.Goal_pre = self.Goallist[self.reach-1]
                self.Goal_far = self.Goallist[self.reach+1]
                self.dist_to_near = math.sqrt((self.Goal_near[0]-self.Goal_pre[0])**2+(self.Goal_near[1]-self.Goal_pre[1])**2)
        if self.reach > 1 and self.reach_trailer < 98:
            if local_rear_near_x < 0 or self.dist_to_near_rear < self.target_range:
                self.reach_trailer += 1 
                self.Goal_near_rear = self.Goallist[self.reach_trailer]
                self.Goal_far_rear = self.Goallist[self.reach_trailer+1]
        if self.timesteps>=25000:
            reward_timeout = -400


        # if self.dist_to_near<self.target_range:# reward for reach target
        #     reward_reach = 100
        reward = reward_timeout+reward_pro+reward_reach+reward_out
        reward = np.float32(reward)
        #print(self.reach,self.reach_trailer)
        # Policy of changing goal point
        return observation,reward,done,{}
    
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        self.timesteps = 0
        self.reach = 0
        self.reach_trailer = 0
        self.car = Truck(self.client)
        self.carId,_ = self.car.get_ids()
        self.done = False
        self.Goallist,self.PointsId,self.planeId,self.Track_dist = Lane(self.client).generate_lane()
        self.k = 6000/self.Track_dist
        #print(self.Goallist)
        self.Goal_near = self.Goallist[0]
        self.Goal_far = self.Goallist[1]
        self.Goal_near_rear = self.Goallist[0]
        self.Goal_far_rear = self.Goallist[1]
        car_ob = self.car.get_observation()
        car_pos = car_ob[:2]
        tire_1_pos = p.getLinkState(self.carId,4,self.client)[0]
        tire_2_pos = p.getLinkState(self.carId,5,self.client)[0]
        tire_3_pos = p.getLinkState(self.carId,7,self.client)[0]
        tire_4_pos = p.getLinkState(self.carId,8,self.client)[0]
        ang = p.getLinkState(self.carId,6,self.client)[1]
        ang = p.getEulerFromQuaternion(ang)
        trailer_ori = (math.cos(ang[2]-1.57), math.sin(ang[2]-1.57))
        tractor_x = (tire_1_pos[0]+tire_2_pos[0])/2
        tractor_y = (tire_1_pos[1]+tire_2_pos[1])/2
        trailer_x = (tire_3_pos[0]+tire_4_pos[0])/2
        trailer_y = (tire_3_pos[1]+tire_4_pos[1])/2
        car_center = (tractor_x,tractor_y)
        trailer_center = (trailer_x,trailer_y)
        #tractor observation
        local_near_x = (self.Goal_near[0]-car_center[0])*car_pos[0]+(self.Goal_near[1]-car_center[1])*car_pos[1]
        local_near_y = -(self.Goal_near[0]-car_center[0])*car_pos[1]+(self.Goal_near[1]-car_center[1])*car_pos[0]
        local_near = (local_near_x,local_near_y)
        local_far_x = (self.Goal_far[0]-car_center[0])*car_pos[0]+(self.Goal_far[1]-car_center[1])*car_pos[1]
        local_far_y = -(self.Goal_far[0]-car_center[0])*car_pos[1]+(self.Goal_far[1]-car_center[1])*car_pos[0]
        local_far = (local_far_x,local_far_y)
        #trailer observation
        local_rear_near_x = (self.Goal_near_rear[0]-trailer_center[0])*trailer_ori[0]+(self.Goal_near_rear[1]-trailer_center[1])*trailer_ori[1]
        local_rear_near_y = -(self.Goal_near_rear[0]-trailer_center[0])*trailer_ori[1]+(self.Goal_near_rear[1]-trailer_center[1])*trailer_ori[0]
        local_rear_near= (local_rear_near_x,local_rear_near_y)
        local_rear_far_x = (self.Goal_far_rear[0]-trailer_center[0])*trailer_ori[0]+(self.Goal_far_rear[1]-trailer_center[1])*trailer_ori[1]
        local_rear_far_y = -(self.Goal_far_rear[0]-trailer_center[0])*trailer_ori[1]+(self.Goal_far_rear[1]-trailer_center[1])*trailer_ori[0]
        local_rear_far= (local_rear_far_x,local_rear_far_y)
        #distance calculation
        self.dist_to_near = math.sqrt(((local_near_x)**2+(local_near_y)**2))
        self.dist_to_far = math.sqrt(((local_far_x)**2+(local_far_y)**2))
        full_obs = car_ob+(self.dist_to_near,)+local_near+(self.dist_to_far,)+local_far
        self.state = full_obs
        return np.array(self.state,dtype=np.float32)
    
    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)

    def seed(self,seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]