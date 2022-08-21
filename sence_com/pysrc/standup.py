#!/usr/bin/env python3
import rospy
import roboticstoolbox
from sence_msgs.msg import Target_Buffer, Target
import time

if __name__ == "__main__" :
    time.sleep(15)
    rospy.init_node('test_stand')
    pub = rospy.Publisher('sence_target', Target_Buffer, queue_size=3)
    i = Target_Buffer()
    i.count = 1
    j1 = Target()
    j1.TARGET_ID = 1
    j1.TARGET_POSITION = 0.71
    j1.TARGET_VELOCITY = 0.3

    pub.publish(i)

    exit()