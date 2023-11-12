#!/usr/bin/env python3

# static cartesian server for performing static cartesian moves of limb contact points
#

import rospy
import actionlib
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction
from cartesian_control_msgs.msg import FollowCartesianTrajectoryGoal
from cartesian_control_msgs.msg import CartesianTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl
import numpy as np


# a class to perform a static gait
class StaticCartesianServer(object):
    def __init__(self):
        # # get the configuration parameters
        # self.gaits = rospy.get_param('~gaits', [])
        # self.joints = rospy.get_param('~joints', [])
        # self.gait_time = rospy.get_param('~gait_time', 0.1)

        self.robot = URDF.from_parameter_server()
        _, tree = treeFromUrdfModel(self.robot)
        chain  = tree.getChain('base_link', 'lf_wrist_contact')
        self.fk_solver = kdl.ChainFkSolverPos_recursive(chain)

        # action client for the joint trajectory action of the controller
        self.jta = actionlib.SimpleActionClient(
            'controllers/lf_cartesian_trajectory/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)

    def random_point(self):
        p_kdl = kdl.Frame()
        joints = kdl.JntArray(3)
        for i in range(3):
            joints[i] = (np.random.random_sample() - 0.5) * 0.8

        self.fk_solver.JntToCart(joints, p_kdl)

        p = CartesianTrajectoryPoint()
        p.pose.position.x = p_kdl.p[0]
        p.pose.position.y = p_kdl.p[1]
        p.pose.position.z = p_kdl.p[2]
        q = kdl.Rotation.GetQuaternion(p_kdl.M)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p

    # run
    def run(self, traj_point: CartesianTrajectoryPoint):
        rospy.loginfo('starting test')

        while not self.jta.wait_for_server(rospy.Duration(0.1)):
            rospy.loginfo_throttle(1.0, 'joint trajectory action server ready')

        # target point
        traj_point.time_from_start = rospy.Duration(0.5)

        # create a joint trajectory goal
        traj_goal = FollowCartesianTrajectoryGoal()
        # add the points
        traj_goal.trajectory.points.append(traj_point)

        # send the goal
        rospy.loginfo('sending goal')
        self.jta.send_goal(traj_goal)
        self.jta.wait_for_result()


# main
if __name__ == '__main__':
    rospy.init_node('static_cartesian_server')

    rospy.loginfo('static cartesian server started')

    scs = StaticCartesianServer()

    # create some test poses
    scs.run(scs.random_point())
    # scs.run(scs.random_point())

    rospy.loginfo('static cartesian server finished')

    exit(0)
