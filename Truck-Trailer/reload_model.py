import gym
from stable_baselines3.common.env_checker import check_env
import truck_trailer
#from Truck-Trailer import truck_trailer
from stable_baselines3 import PPO
from stable_baselines3.ppo.policies import MlpPolicy

env = gym.make('TruckTrailer-v0',render = False)
model = PPO.load("model_busmodel_5000000")
model.learn(total_timesteps=5000000)
model.save("reload")