from fileinput import filename
from multiprocessing.connection import Client
import pybullet as p
import os

class track:
    def __init__(self,client):
        self.client = client
        #f_name = os.path.join(os.path.dirname(__file__),"straight_line_scen.urdf")
        self.plane= p.loadURDF("./Truck-Trailer/truck_trailer/resources/simpleplane.urdf",
                               basePosition = [0,0,0],
                               physicsClientId = client)
        
    def get_ids(self):
        return self.plane,self.client