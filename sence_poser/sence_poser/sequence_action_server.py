import threading

import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from action_msgs.msg import GoalStatus

from sence_msgs.action import StaticPose, PoseSequence

from .sence_poses import jointNames, poses, sequences, loops, poseSec, poseNano


class SequenceActionServer(Node):
    def __init__(self):
        # start the node
        super().__init__('sequence_action_server')

        # start the joint trajectory action client
        self.jta_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

        # start the pose action server
        self.pose_action_server = ActionServer(
            self,
            StaticPose,
            'static_pose',
            self.pose_execute_callback)
        # print out the available poses
        self.get_logger().info('available poses: ' + str(list(poses.keys()))[1:-1])

        # start the sequence action server
        self.sequence_action_server = ActionServer(
            self,
            PoseSequence,
            'pose_sequence',
            self.sequence_execute_callback)
        # print out the available sequences
        self.get_logger().info('available sequences: ' + str(list(sequences.keys()))[1:-1])

        # start the loop action server
        self.sequence_action_server = ActionServer(
            self,
            PoseSequence,
            'sequence_loop',
            self.loop_execute_callback)
        # print out the available loops
        self.get_logger().info('available loops: ' + str(list(loops))[1:-1])

        self.get_logger().info('Waiting for JT action server...')
        self.jta_client.wait_for_server()



    # the callback to use when a single pose action is called
    def pose_execute_callback(self, goal_handle):
        self.get_logger().info('Moving to pose: ' + goal_handle.request.pose)

        goal_pose = goal_handle.request.pose

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = jointNames
        
        point = JointTrajectoryPoint()
        point.positions = poses[goal_pose]
        point.time_from_start.sec = poseSec
        point.time_from_start.nanosec = poseNano
        goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self.jta_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        goal_handle.succeed()
        result = StaticPose.Result()
        result.success = True #todo test for success

        return result


    # the callback to use when a multi point action is called
    def sequence_execute_callback(self, goal_handle):
        self.get_logger().info('Running Sequence: ' + goal_handle.request.sequence)

        goal_sequence = goal_handle.request.sequence

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = jointNames
        
        for (index, pose) in list(enumerate(sequences[goal_sequence])):
            self.get_logger().info('adding trajectory point ' + str(index) + " " "pose " + str(pose))
            point = JointTrajectoryPoint()
            point.positions = poses[pose]
            point.time_from_start.sec = poseSec
            point.time_from_start.nanosec = poseNano + index * poseNano
            goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self.jta_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        goal_handle.succeed()
        result = PoseSequence.Result()
        result.success = True #todo test for success

        return result
    
    # the callback to use when a looping multi point action is called
    def loop_execute_callback(self, goal_handle):
        self.get_logger().info('Running Loop: ' + goal_handle.request.sequence)

        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()

        one_second = self.create_rate(2.1)
        
        while True:
            try:
                goal_sequence = goal_handle.request.sequence

                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory = JointTrajectory()
                goal_msg.trajectory.joint_names = jointNames
                
                for (index, pose) in list(enumerate(sequences[goal_sequence])):
                    self.get_logger().info('adding trajectory point ' + str(index) + " " "pose " + str(pose))
                    point = JointTrajectoryPoint()
                    point.positions = poses[pose]
                    point.time_from_start.sec = poseSec
                    point.time_from_start.nanosec = poseNano + index * poseNano
                    goal_msg.trajectory.points.append(point)

                self.get_logger().info('Sending goal request...')

                self._send_goal_future = self.jta_client.send_goal_async(goal_msg)                
                one_second.sleep()
                        
                    
            except KeyboardInterrupt:
                break

        goal_handle.succeed()
        result = PoseSequence.Result()
        result.success = True #todo test for success

        return result


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Action ended with result code: ' + str(result.error_code))
        


def main(args=None):
    rclpy.init(args=args)

    sequence_action_server = SequenceActionServer()

    rclpy.spin(sequence_action_server)


if __name__ == '__main__':
    main()