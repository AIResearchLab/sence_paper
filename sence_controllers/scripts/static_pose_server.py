#!/usr/bin/env python3

# a static manuever service to perform pre-defined poses and sequences of poses in joint space
#

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String

from sence_controllers.msg import StaticPoseAction
from sence_controllers.msg import StaticPoseGoal


# a class to perform a sequence of poses
class StaticPoseServer(object):
    def __init__(self):
        # get the configuration parameters
        self.poses = rospy.get_param('~poses', [])
        self.sequences = rospy.get_param('~sequences', [])
        self.joints = rospy.get_param('~joints', [])
        self.seq_time = rospy.get_param('~sequence_time', 0.1)

        rospy.loginfo('static pose sequences: %s', self.sequences)

        # action server for the joint trajectory action of the controller
        self.jta = actionlib.SimpleActionClient(
            'controllers/group_position_trajectory/follow_joint_trajectory', FollowJointTrajectoryAction)

        # create a static sequence action server
        self.static_sequence_action = actionlib.SimpleActionServer(
            'static_pose', StaticPoseAction, self.static_sequence_action_cb, False)

        # start the action server
        self.static_sequence_action.start()

    # callbacks
    # action cb
    def static_sequence_action_cb(self, goal):
        rospy.loginfo('new static pose action: %s', goal.pose)

        # accept a sequence action
        # self.static_sequence_action.accept_new_goal()

        # wait for the action server to become available
        while not self.jta.wait_for_server(rospy.Duration(0.1)):
            rospy.loginfo_throttle(1.0, 'joint trajectory action server ready')

        # get poses in sequence from parameter server
        poses = rospy.get_param('~' + goal.pose, [])

        rospy.loginfo('static pose sequence: %s', poses)

        # for each pose in the sequence do the pose
        for pose in poses:
            # preemt the action
            if self.static_sequence_action.is_preempt_requested():
                self.static_sequence_action.set_preempted()
                break

            # get pose data from parameter server
            positions = rospy.get_param('~' + pose, [])

            # create a goal object for the pose
            fjt_goal = FollowJointTrajectoryGoal()
            fjt_goal.trajectory.joint_names = self.joints
            # append a point to the trajectory
            point = JointTrajectoryPoint()
            point.positions = positions
            # point.velocities = [self.vel for p in positions]
            # point.accelerations = [0.0 for p in positions]
            point.time_from_start = rospy.Duration(self.seq_time)
            fjt_goal.trajectory.points.append(point)

            rospy.loginfo('sending trajectory for static pose: %s', pose)
            # rospy.loginfo('trajectory: %s', fjt_goal)

            # send the goal pose
            self.jta.send_goal(fjt_goal)
            while not self.jta.wait_for_result():
                rospy.loginfo_throttle(
                    1.0, 'waiting for static pose action to complete')

        # complete the action
        self.static_sequence_action.set_succeeded()


# main
if __name__ == '__main__':
    rospy.init_node('static_pose_server')

    rospy.loginfo('static pose server started')

    sps = StaticPoseServer()

    rospy.spin()

    exit(0)
