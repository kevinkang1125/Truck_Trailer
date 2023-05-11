import pybullet as p
import os


class Goal:
    def __init__(self,base):
        p.loadURDF(fileName='./models/simplegoal.urdf',
                   basePosition=[base[0], base[1], 0.025])
                   #physicsClientId=client)


