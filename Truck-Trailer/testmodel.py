import truck_trailer
import pybullet as p
from time import sleep
import pybullet_data
from truck_trailer.resources.lane import Lane

client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

wayponitscor, waypoints = Lane(client).generate_lane()
sleep(1000)