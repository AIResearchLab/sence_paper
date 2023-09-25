#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/sence_hardware/controller/joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('ZeroTalkerNode', anonymous = True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        hello_str = "now sending one (time %s)" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(1.0)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass