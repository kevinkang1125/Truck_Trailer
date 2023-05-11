from time import sleep
import pybullet as p
import pybullet_data
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
bus = p.loadURDF('./models/TT/urdf/TT.urdf')# fill in model name 
number_of_joints = p.getNumJoints(bus)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(bus,joint_number)
    print(info[0],":",info[1])
linvel,angvel = p.getBaseVelocity(bus)
vel = p.getBaseVelocity(bus)[0][0:2]
print(linvel,angvel)
print(vel)