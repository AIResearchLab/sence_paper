import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)

p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")

robot = p.loadURDF("sence_main_description/urdf/sence_comb.urdf",[0, 0, 0.3], useFixedBase=0)

position, orientation = p.getBasePositionAndOrientation(robot)
print("The robot position is {}".format(position))
print("The robot orientation (x, y, z, w) is {}".format(orientation))
nb_joints = p.getNumJoints(robot)
print("The robot is made of {} joints.".format(nb_joints))
p.setGravity(0, 0, -9.81)

joints_index_list = range(nb_joints)
joints_state_list = p.getJointStates(robot, joints_index_list)
link_state_list = p.getLinkState(robot, nb_joints-1)
p.resetDebugVisualizerCamera( cameraDistance=0.35, cameraYaw=-45, cameraPitch=-20, cameraTargetPosition=[0,0,0.2])
print(joints_state_list)
time.sleep(2)
n = 0
for _ in range(10000000):
    n = n + 1
    p.stepSimulation()
    time.sleep(1./90.)
    # if n == (100):
    #Joint order [chassis,backright(-),backleft(+),frontright(+),frontleft(-)]
    #Leave the first value at 0 for the chassis, and the first value of each leg at 0 as it is the base link
    p.setJointMotorControlArray(robot, joints_index_list, p.POSITION_CONTROL, targetPositions=[0,0, -0.7854, -1.5708, 1.5708, 0, 0.7854, 1.5708, 1.5708, 0, 0.7854, 1.5708, 1.5708, 0, -0.7854, -1.5708, 1.5708])
    


p.disconnect()


'''
import pybullet as p
import pybullet_utils.bullet_client as bc


class Foo:
    def __init__(self, counter):
        self.physicsClient = bc.BulletClient(connection_mode=p.DIRECT)

    def setGravity(self):
        self.physicsClient.setGravity(0, 0, -9.81)


foo1 = Foo(1)
foo2 = Foo(2)
foo1.setGravity()
foo2.setGravity()

print("Adress of  foo1 bullet client 1 : " + str(foo1.physicsClient))
print("Adress of foo2 bullet client 2  : " + str(foo2.physicsClient))
'''
