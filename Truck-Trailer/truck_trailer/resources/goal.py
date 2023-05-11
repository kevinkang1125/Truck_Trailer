import pybullet as p
import os


class Goal:
    def __init__(self,client,base):
        #f_name = os.path.join(os.path.dirname(__file__), 'simplegoal.urdf')
        self.client = client
        self.target =p.loadURDF(fileName="./Truck-Trailer/truck_trailer/resources/simplegoal.urdf",
                                basePosition=[base[0], base[1], 0.025],
                                physicsClientId=self.client)
    
    def get_ids(self):
        
        return self.target,self.client



