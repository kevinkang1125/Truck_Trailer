from time import sleep
import pybullet as p
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
bus = p.loadURDF('./models/test.urdf',[10,0,3])
number_of_joints = p.getNumJoints(bus)
# for joint_number in range(number_of_joints):
#     info = p.getJointInfo(bus,joint_number)
#     print(info[0],":",info[1])
angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
throttle = p.addUserDebugParameter('Throttle', 0, 20, 0)
plane = p.loadURDF('./models/straight_line_scen.urdf')

sleep(3)
wheel_indices = [1, 3, 4, 5]
hinge_indices = [0, 2]
while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    for joint_index in wheel_indices:
        p.setJointMotorControl2(bus, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttle)
    for joint_index in hinge_indices:
        p.setJointMotorControl2(bus, joint_index,
                                p.POSITION_CONTROL, 
                                targetPosition=user_angle)
    p.stepSimulation()