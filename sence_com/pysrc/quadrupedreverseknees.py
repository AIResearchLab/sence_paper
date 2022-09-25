#!/usr/bin/env python3
import rospy
import roboticstoolbox
from sence_msgs.msg import Target_Buffer, Target
import time
from time import sleep

if __name__ == "__main__" :
    i = Target_Buffer()
    j1 = Target()
    j1.TARGET_ID = 1
    j1.TARGET_POSITION = 0
    j1.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j1)
    j2 = Target()
    j2.TARGET_ID = 2
    j2.TARGET_POSITION = 0
    j2.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j2)
    j3 = Target()
    j3.TARGET_ID = 3
    j3.TARGET_POSITION = 0
    j3.TARGET_VELOCITY = 0.5
    i.Buffer_To_Send.append(j3)
    j4 = Target()
    j4.TARGET_ID = 4
    j4.TARGET_POSITION = 0
    j4.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j4)
    j5 = Target()
    j5.TARGET_ID = 5
    j5.TARGET_POSITION = 0
    j5.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j5)
    j6 = Target()
    j6.TARGET_ID = 6
    j6.TARGET_POSITION = 0
    j6.TARGET_VELOCITY = 0.5
    i.Buffer_To_Send.append(j6)
    j7 = Target()
    j7.TARGET_ID = 7
    j7.TARGET_POSITION = 0
    j7.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j7)
    j8 = Target()
    j8.TARGET_ID = 8
    j8.TARGET_POSITION = 0
    j8.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j8)
    j9 = Target()
    j9.TARGET_ID = 9
    j9.TARGET_POSITION = 0
    j9.TARGET_VELOCITY = 0.5
    i.Buffer_To_Send.append(j9)
    j10 = Target()
    j10.TARGET_ID = 10
    j10.TARGET_POSITION = 0
    j10.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j10)
    j11 = Target()
    j11.TARGET_ID = 11
    j11.TARGET_POSITION = 0
    j11.TARGET_VELOCITY = 0.4
    i.Buffer_To_Send.append(j11)
    j12 = Target()
    j12.TARGET_ID = 12
    j12.TARGET_POSITION = 0
    j12.TARGET_VELOCITY = 0.5
    i.Buffer_To_Send.append(j12)
    i.count = len(i.Buffer_To_Send)

    pub = rospy.Publisher('/sence_target', Target_Buffer, queue_size=3)

    while(True):
        h = input('input command and press enter: ')
        rospy.init_node('test_stand')
        if h == '0': #Reset to zero position outside of startup
            i.Buffer_To_Send[0].TARGET_POSITION = 0
            i.Buffer_To_Send[1].TARGET_POSITION = 0
            i.Buffer_To_Send[2].TARGET_POSITION = 0
            i.Buffer_To_Send[3].TARGET_POSITION = 0
            i.Buffer_To_Send[4].TARGET_POSITION = 0
            i.Buffer_To_Send[5].TARGET_POSITION = 0
            i.Buffer_To_Send[6].TARGET_POSITION = 0
            i.Buffer_To_Send[7].TARGET_POSITION = 0
            i.Buffer_To_Send[8].TARGET_POSITION = 0
            i.Buffer_To_Send[9].TARGET_POSITION = 0
            i.Buffer_To_Send[10].TARGET_POSITION = 0
            i.Buffer_To_Send[11].TARGET_POSITION = 0
            pub.publish(i)
            sleep(4)
        if h == '1': #Reset to zero position on startup and initial prestand
            i.Buffer_To_Send[0].TARGET_POSITION = 0.21
            i.Buffer_To_Send[3].TARGET_POSITION = -0.21
            i.Buffer_To_Send[6].TARGET_POSITION = 0.21
            i.Buffer_To_Send[9].TARGET_POSITION = -0.21
            pub.publish(i)
            sleep(2)
            i.Buffer_To_Send[1].TARGET_POSITION = -1.57
            i.Buffer_To_Send[2].TARGET_POSITION = 2.09
            i.Buffer_To_Send[4].TARGET_POSITION = 1.57
            i.Buffer_To_Send[5].TARGET_POSITION = 2.09
            i.Buffer_To_Send[7].TARGET_POSITION = -1.57
            i.Buffer_To_Send[8].TARGET_POSITION = 2.09
            i.Buffer_To_Send[10].TARGET_POSITION = 1.57
            i.Buffer_To_Send[11].TARGET_POSITION = 2.09
            pub.publish(i)
        elif h == '2': #stand
            i.Buffer_To_Send[0].TARGET_POSITION = 2.15
            i.Buffer_To_Send[3].TARGET_POSITION = -2.15
            i.Buffer_To_Send[6].TARGET_POSITION = 2.15
            i.Buffer_To_Send[9].TARGET_POSITION = -2.15
            pub.publish(i)
            sleep(2)
            i.Buffer_To_Send[2].TARGET_POSITION = 1.25
            i.Buffer_To_Send[5].TARGET_POSITION = 1.25
            i.Buffer_To_Send[8].TARGET_POSITION = 1.25
            i.Buffer_To_Send[11].TARGET_POSITION = 1.25    
            pub.publish(i)
        elif h == '3': #Single walk cycle
            #Temp setup for movement, set to end of first pull push setup
            i.Buffer_To_Send[2].TARGET_POSITION = 1.48353 #reach for pull
            i.Buffer_To_Send[8].TARGET_POSITION = 1.48353 #reach for pull
            i.Buffer_To_Send[5].TARGET_POSITION = 1.25
            i.Buffer_To_Send[11].TARGET_POSITION = 0.959931
            i.Buffer_To_Send[0].TARGET_POSITION = 1.396263 #reach for pull
            i.Buffer_To_Send[6].TARGET_POSITION = 2.268928 #reach for pull
            i.Buffer_To_Send[3].TARGET_POSITION = -2.268928
            i.Buffer_To_Send[9].TARGET_POSITION = -2.007129
            pub.publish(i)
            sleep(0.4)
            #Feet down
            i.Buffer_To_Send[2].TARGET_POSITION = 0.959931 #reach for pull
            i.Buffer_To_Send[0].TARGET_POSITION = 2.007129 #reach for pull
            i.Buffer_To_Send[8].TARGET_POSITION = 1.22173 #reach for pull
            pub.publish(i)
            sleep(0.75)
            #stride 2
            i.Buffer_To_Send[5].TARGET_POSITION = 1.48353 #reach for pull
            i.Buffer_To_Send[11].TARGET_POSITION = 1.48353 #reach for pull
            i.Buffer_To_Send[2].TARGET_POSITION = 1.25
            i.Buffer_To_Send[8].TARGET_POSITION = 0.959931
            i.Buffer_To_Send[3].TARGET_POSITION = -1.396263 #reach for pull
            i.Buffer_To_Send[9].TARGET_POSITION = -2.268928 #reach for pull
            i.Buffer_To_Send[0].TARGET_POSITION = 2.268928
            i.Buffer_To_Send[6].TARGET_POSITION = 2.007129
            pub.publish(i)
            sleep(0.4)
            #Feet down
            i.Buffer_To_Send[5].TARGET_POSITION = 0.959931 #reach for pull
            i.Buffer_To_Send[3].TARGET_POSITION = -2.007129 #reach for pull
            i.Buffer_To_Send[11].TARGET_POSITION = 1.22173 #reach for pull
            pub.publish(i)


        elif h == '4': #Single walk cycle
            #Temp setup for movement, set to end of first pull push setup
            cc=0
            while cc < 3:
                i.Buffer_To_Send[2].TARGET_POSITION = 1.48353 #reach for pull
                i.Buffer_To_Send[8].TARGET_POSITION = 1.48353 #reach for pull
                i.Buffer_To_Send[5].TARGET_POSITION = 1.25
                i.Buffer_To_Send[11].TARGET_POSITION = 0.959931
                i.Buffer_To_Send[0].TARGET_POSITION = 1.396263 #reach for pull
                i.Buffer_To_Send[6].TARGET_POSITION = 2.268928 #reach for pull
                i.Buffer_To_Send[3].TARGET_POSITION = -2.268928
                i.Buffer_To_Send[9].TARGET_POSITION = -2.007129
                pub.publish(i)
                sleep(0.4)
                #Feet down
                i.Buffer_To_Send[2].TARGET_POSITION = 0.959931 #reach for pull
                i.Buffer_To_Send[0].TARGET_POSITION = 2.007129 #reach for pull
                i.Buffer_To_Send[8].TARGET_POSITION = 1.22173 #reach for pull
                pub.publish(i)
                sleep(0.75)
                #stride 2
                i.Buffer_To_Send[5].TARGET_POSITION = 1.48353 #reach for pull
                i.Buffer_To_Send[11].TARGET_POSITION = 1.48353 #reach for pull
                i.Buffer_To_Send[2].TARGET_POSITION = 1.25
                i.Buffer_To_Send[8].TARGET_POSITION = 0.959931
                i.Buffer_To_Send[3].TARGET_POSITION = -1.396263 #reach for pull
                i.Buffer_To_Send[9].TARGET_POSITION = -2.268928 #reach for pull
                i.Buffer_To_Send[0].TARGET_POSITION = 2.268928
                i.Buffer_To_Send[6].TARGET_POSITION = 2.007129
                pub.publish(i)
                sleep(0.4)
                #Feet down
                i.Buffer_To_Send[5].TARGET_POSITION = 0.959931 #reach for pull
                i.Buffer_To_Send[3].TARGET_POSITION = -2.007129 #reach for pull
                i.Buffer_To_Send[11].TARGET_POSITION = 1.22173 #reach for pull
                pub.publish(i)
                sleep(0.75)
                cc += 1
            #end stride, return to base pose
            i.Buffer_To_Send[2].TARGET_POSITION = 1.48353 #reach for pull
            i.Buffer_To_Send[8].TARGET_POSITION = 1.48353 #reach for pull
            i.Buffer_To_Send[5].TARGET_POSITION = 1.25
            i.Buffer_To_Send[11].TARGET_POSITION = 1.25
            i.Buffer_To_Send[0].TARGET_POSITION = 1.396263 #reach for pull
            i.Buffer_To_Send[6].TARGET_POSITION = 2.268928 #reach for pull
            i.Buffer_To_Send[3].TARGET_POSITION = -2.15
            i.Buffer_To_Send[9].TARGET_POSITION = -2.15
            pub.publish(i)
            sleep(0.4)
            #Feet down
            i.Buffer_To_Send[2].TARGET_POSITION = 1.25 #reach for pull
            i.Buffer_To_Send[0].TARGET_POSITION = 2.15 #reach for pull
            i.Buffer_To_Send[8].TARGET_POSITION = 1.25 #reach for pull
            i.Buffer_To_Send[6].TARGET_POSITION = 2.15


            pub.publish(i)
            
        if h=='q':
            exit()