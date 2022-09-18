import pybullet as p
import pybullet_data
import sence_server
import time
import rospy
from sence_msgs.msg import Target_Buffer, Target

sence_server.load_server()
p.connect(p.SHARED_MEMORY)

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

def callback(data):
    l = len(data.Buffer_To_Send)
    i = 0
    REMAP_ID = [10,11,12,14,15,16,6,7,8,2,3,4]
    while i < l:
        print("Command received:")
        TARGET_ID_ORIGIN = data.Buffer_To_Send[i].TARGET_ID
        TARGET_ID = REMAP_ID[TARGET_ID_ORIGIN-1]
        print("ID:",data.Buffer_To_Send[i].TARGET_ID)
        TARGET_POSITION = data.Buffer_To_Send[i].TARGET_POSITION
        print("POS:",data.Buffer_To_Send[i].TARGET_POSITION)
        TARGET_VELOCITY = data.Buffer_To_Send[i].TARGET_VELOCITY
        print("VEL:",data.Buffer_To_Send[i].TARGET_VELOCITY)
        sence_server.command_actuator(robot,TARGET_ID,TARGET_POSITION,TARGET_VELOCITY)
        i+=1
    #Joint order [chassis(0),backright(2-4)(-),backleft(6-8)(+)
    #frontright(10-12)(+),frontleft(14-16)(-)]
    #first value at 0 for the chassis, and the first of each leg as it is the base link
def listener():
    rospy.init_node('sim_command', anonymous=True)

    rospy.Subscriber("/sence_target", Target_Buffer, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
 
if __name__ == '__main__':
    listener()