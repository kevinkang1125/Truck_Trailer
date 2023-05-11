import pybullet as p
import os


class Road:
    def __init__(self,base):
        p.loadURDF(fileName='./models/simpleroad.urdf',
                   basePosition=[base[0], base[1]-10, 0.025])
                   #physicsClientId=client)
        p.loadURDF(fileName='./models/simpleroad.urdf',
                    basePosition=[base[0], base[1]+10, 0.025])
            #physicsClientId=client)


