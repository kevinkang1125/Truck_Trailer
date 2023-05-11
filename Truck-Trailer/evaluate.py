import time

import gym
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import truck_trailer
#from Truck-Trailer import truck_trailer
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.ppo.policies import MlpPolicy

env = gym.make('TruckTrailer-v1',render = True)
#env = env(render = False)
model = PPO.load("./log/best_model")#model name
p.resetDebugVisualizerCamera(cameraDistance = 180,cameraYaw = -90,cameraPitch = -89, cameraTargetPosition = [100,0,0])
#obs = env.reset()
# for i in range (1000000):
#     action,_state = model.predict(obs)
#     obs,rewards,done,info = env.step(action)
#     print(action)

for i in range (10):
    # obs = env.reset()
    done = False
    # while not done:
    obs = env.reset()
    #plt.axis([0,2000,-1,1])
    step = 0
    record_steer = []
    record_vel = []
 
    while not done:
        action,_state = model.predict(obs)
        obs,rewards,done,info = env.step(action)
        #time.sleep(0.1)
        #print(obs)
        x = action[0]
        y = action[1]
        record_vel.append(10*x+10)
        record_steer.append(0.55*y)
    

 
        
    fig,(ax1,ax2)=plt.subplots(1,2)
    ax1.plot(record_vel)
    ax1.set_title('velocity command')
    ax2.plot(record_steer)
    ax2.set_title('steer command')
    plt.show()
    i +=1
    