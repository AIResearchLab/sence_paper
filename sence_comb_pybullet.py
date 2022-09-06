import pybullet as p
import pybullet_data
import sence_server
import time

sence_server.load_server()
p.connect(p.SHARED_MEMORY)

def command_actuator(M_ID,target_position,target_velocity):
    p.setJointMotorControl2(
        bodyIndex=robot,
        jointIndex=M_ID,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target_position,
        maxVelocity=target_velocity,
        force=2.5) 
    return

p.setAdditionalSearchPath(pybullet_data.getDataPath())

robot = p.loadURDF("sence_main_description/urdf/sence_comb.urdf",[0, 0, 0.3], useFixedBase=0)

position, orientation = p.getBasePositionAndOrientation(robot)
print("The robot position is {}".format(position))
print("The robot orientation (x, y, z, w) is {}".format(orientation))
nb_joints = p.getNumJoints(robot)
print("The robot is made of {} joints.".format(nb_joints))


joints_index_list = range(nb_joints)
joints_state_list = p.getJointStates(robot, joints_index_list)
link_state_list = p.getLinkState(robot, nb_joints-1)
p.resetDebugVisualizerCamera( cameraDistance=0.35, cameraYaw=-45, cameraPitch=-20, cameraTargetPosition=[0,0,0.2])
time.sleep(2)
vel_radpm = 2.30383
n = 0
for _ in range(10000000):
    n = n + 1
    time.sleep(0.01)
    if n == (500):
        sence_server.command_actuator(robot,2,-0.7854,vel_radpm)
        sence_server.command_actuator(robot,3,-1.5708,vel_radpm)
        sence_server.command_actuator(robot,4,1.5708,vel_radpm)
        sence_server.command_actuator(robot,6,0.7854,vel_radpm)
        sence_server.command_actuator(robot,7,1.5708,vel_radpm)
        sence_server.command_actuator(robot,8,1.5708,vel_radpm)
        sence_server.command_actuator(robot,10,0.7854,vel_radpm)
        sence_server.command_actuator(robot,11,1.5708,vel_radpm)
        sence_server.command_actuator(robot,12,1.5708,vel_radpm)
        sence_server.command_actuator(robot,14,-0.7854,vel_radpm)
        sence_server.command_actuator(robot,15,-1.5708,vel_radpm)
        sence_server.command_actuator(robot,16,1.5708,vel_radpm)

    #Joint order [chassis(0),backright(2-4)(-),backleft(6-8)(+),frontright(10-12)(+),frontleft(14-16)(-)]
    #Leave the first value at 0 for the chassis, and the first value of each leg at 0 as it is the base link