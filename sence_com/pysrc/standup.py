#!/usr/bin/env python3
import rospy
import roboticstoolbox
from sence_msgs.msg import Target_Buffer, Target
import time

if __name__ == "__main__" :
    i = Target_Buffer()
    j1 = Target()
    j1.TARGET_ID = 1
    j1.TARGET_POSITION = 0.71
    j1.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j1)
    j2 = Target()
    j2.TARGET_ID = 2
    j2.TARGET_POSITION = -1.58
    j2.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j2)
    j3 = Target()
    j3.TARGET_ID = 3
    j3.TARGET_POSITION = 1.05
    j3.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j3)
    j4 = Target()
    j4.TARGET_ID = 4
    j4.TARGET_POSITION = -0.71
    j4.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j4)
    j5 = Target()
    j5.TARGET_ID = 5
    j5.TARGET_POSITION = 1.58
    j5.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j5)
    j6 = Target()
    j6.TARGET_ID = 6
    j6.TARGET_POSITION = 1.05
    j6.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j6)
    j7 = Target()
    j7.TARGET_ID = 7
    j7.TARGET_POSITION = 0.71
    j7.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j7)
    j8 = Target()
    j8.TARGET_ID = 8
    j8.TARGET_POSITION = -1.58
    j8.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j8)
    j9 = Target()
    j9.TARGET_ID = 9
    j9.TARGET_POSITION = 1.05
    j9.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j9)
    j10 = Target()
    j10.TARGET_ID = 10
    j10.TARGET_POSITION = -0.71
    j10.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j10)
    j11 = Target()
    j11.TARGET_ID = 11
    j11.TARGET_POSITION = 1.58
    j11.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j11)
    j12 = Target()
    j12.TARGET_ID = 12
    j12.TARGET_POSITION = 1.05
    j12.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j12)
    i.count = len(i.Buffer_To_Send)

    pub = rospy.Publisher('/sence_target', Target_Buffer, queue_size=3)

    while(True):
        h = input('press enter')
        rospy.init_node('test_stand')
        if h == '1':
            i.Buffer_To_Send[0].TARGET_POSITION = 0.71
            i.Buffer_To_Send[3].TARGET_POSITION = -0.71
            i.Buffer_To_Send[6].TARGET_POSITION = 0.71
            i.Buffer_To_Send[9].TARGET_POSITION = -0.71    
            pub.publish(i)
        elif h == '2':
            i.Buffer_To_Send[0].TARGET_POSITION = 1.58
            i.Buffer_To_Send[3].TARGET_POSITION = -1.58
            i.Buffer_To_Send[6].TARGET_POSITION = 1.58
            i.Buffer_To_Send[9].TARGET_POSITION = -1.58    
            pub.publish(i)
        if h=='q':
            exit()