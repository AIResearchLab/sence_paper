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

def controller():
    rospy.init_node('sim_command', anonymous=True)

    sub = rospy.Subscriber('/sence_target', Target_Buffer, callback)
    pub = rospy.Publisher('/sence_sim_telem', Target_Buffer, queue_size=3)
    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        REMAP_ID = [10,11,12,14,15,16,6,7,8,2,3,4]
        t = Target_Buffer()
        j1 = Target()
        j1.TARGET_ID = 1
        joint=p.getJointState(robot, REMAP_ID[j1.TARGET_ID-1])
        j1.TARGET_POSITION = joint[0]
        j1.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j1)
        j2 = Target()
        j2.TARGET_ID = 2
        joint=p.getJointState(robot, REMAP_ID[j2.TARGET_ID-1])
        j2.TARGET_POSITION = joint[0]
        j2.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j2)
        j3 = Target()
        j3.TARGET_ID = 3
        joint=p.getJointState(robot, REMAP_ID[j3.TARGET_ID-1])
        j3.TARGET_POSITION = joint[0]
        j3.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j3)
        j4 = Target()
        j4.TARGET_ID = 4
        joint=p.getJointState(robot, REMAP_ID[j4.TARGET_ID-1])
        j4.TARGET_POSITION = joint[0]
        j4.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j4)
        j5 = Target()
        j5.TARGET_ID = 5
        joint=p.getJointState(robot, REMAP_ID[j5.TARGET_ID-1])
        j5.TARGET_POSITION = joint[0]
        j5.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j5)
        j6 = Target()
        j6.TARGET_ID = 6
        joint=p.getJointState(robot, REMAP_ID[j6.TARGET_ID-1])
        j6.TARGET_POSITION = joint[0]
        j6.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j6)
        j7 = Target()
        j7.TARGET_ID = 7
        joint=p.getJointState(robot, REMAP_ID[j7.TARGET_ID-1])
        j7.TARGET_POSITION = joint[0]
        j7.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j7)
        j8 = Target()
        j8.TARGET_ID = 8
        joint=p.getJointState(robot, REMAP_ID[j8.TARGET_ID-1])
        j8.TARGET_POSITION = joint[0]
        j8.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j8)
        j9 = Target()
        j9.TARGET_ID = 9
        joint=p.getJointState(robot, REMAP_ID[j9.TARGET_ID-1])
        j9.TARGET_POSITION = joint[0]
        j9.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j9)
        j10 = Target()
        j10.TARGET_ID = 10
        joint=p.getJointState(robot, REMAP_ID[j10.TARGET_ID-1])
        j10.TARGET_POSITION = joint[0]
        j10.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j10)
        j11 = Target()
        j11.TARGET_ID = 11
        joint=p.getJointState(robot, REMAP_ID[j11.TARGET_ID-1])
        j11.TARGET_POSITION = joint[0]
        j11.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j11)
        j12 = Target()
        j12.TARGET_ID = 12
        joint=p.getJointState(robot, REMAP_ID[j12.TARGET_ID-1])
        j12.TARGET_POSITION = joint[0]
        j12.TARGET_VELOCITY = joint[1]
        t.Buffer_To_Send.append(j12)
        t.count = len(t.Buffer_To_Send)
        pub.publish(t)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInitException:
        pass