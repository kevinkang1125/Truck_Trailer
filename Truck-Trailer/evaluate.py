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
import math

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
    record_tractor_x = []
    record_tractor_y = []
    record_trailer_x = []
    record_trailer_y = []
    trailer_deviation = []
    tractor_deviation = []
    m = 0
    while not done:
        action,_state = model.predict(obs)
        obs,rewards,done,info = env.step(action)
        #time.sleep(0.1)
        #print(obs)
        x = action[0]
        y = action[1]
        vel_x = obs[2]
        vel_y = obs[3]
        vel = math.sqrt(vel_x**2+vel_y**2)
        record_vel.append(vel)
        record_steer.append(0.55*y)
        record_tractor_x.append(info["Tractor_x"])
        record_tractor_y.append(info["Tractor_y"])
        record_trailer_x.append(info["trailer_x"])
        record_trailer_y.append(info["trailer_y"])
        trailer_deviation.append(info["dev_trailer"])
        tractor_deviation.append(info["dev_tractor"])
        m += 1
    # np.savetxt("results/tractor_x{}.txt".format(i),record_tractor_x)
    # np.savetxt("results/tractor_y{}.txt".format(i),record_tractor_y)
    # np.savetxt("results/trailer_x{}.txt".format(i),record_trailer_x)    
    # np.savetxt("results/trailer_y{}.txt".format(i),record_trailer_y)
    # np.savetxt("results/velocity{}.txt".format(i),record_vel)
    # np.savetxt("results/steer{}.txt".format(i),record_steer)
    
    trailer_ave = sum(trailer_deviation)/len(trailer_deviation)
    tractor_ave = sum(tractor_deviation)/len(tractor_deviation) 
    print(trailer_ave,tractor_ave)   
    fig,(ax1,ax2)=plt.subplots(1,2)
    ax1.plot(record_vel)
    ax1.set_title('velocity command')
    ax2.plot(record_steer)
    ax2.set_title('steer command')
    plt.show()
    i +=1
    