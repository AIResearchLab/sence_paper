# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import asyncio

from action_msgs.msg import GoalStatus

from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MinimalActionClientAsyncIO(Node):

    def __init__(self):
        super().__init__('minimal_action_client_asyncio')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.msg = self.build_goal_msg()

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.error))

    def build_goal_msg(self):
        # # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['fl_joint1', 'fl_joint2', 'fl_joint3']

        points = [
            {'positions': [0.0, 0.0, 0.0], 'velocities': [1.0, 1.0, 1.0], 'time_from_start': Duration(sec=1)},
            {'positions': [1.0, 1.0, 1.0], 'velocities': [-3.0, -3.0, -3.0], 'time_from_start': Duration(sec=2)},
        ]

        for point_data in points:
            point_msg = JointTrajectoryPoint()
            point_msg.positions = point_data['positions']
            point_msg.velocities = point_data['velocities']
            point_msg.time_from_start = point_data['time_from_start']
            goal_msg.trajectory.points.append(point_msg)

        return goal_msg

    async def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = self.msg

        self.get_logger().info('Sending goal request...')

        goal_handle = await self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        res = await goal_handle.get_result_async()
        result = res.result
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.error_code))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        return result, status


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop):

    logger = rclpy.logging.get_logger('minimal_action_client')

    # init ros2
    rclpy.init(args=args)

    # create node
    action_client = MinimalActionClientAsyncIO()

    # start spinning
    spin_task = loop.create_task(spinning(action_client))

    # Parallel example
    # execute goal request and schedule in loop
    # my_task1 = loop.create_task(action_client.send_goal())
    # my_task2 = loop.create_task(action_client.send_goal())

    # # glue futures together and wait
    # wait_future = asyncio.wait([my_task1, my_task2])
    # # run event loop
    # finished, unfinished = await wait_future
    # logger.info(f'unfinished: {len(unfinished)}')
    # for task in finished:
    #     logger.info('result {} and status flag {}'.format(*task.result()))

    # Sequence
    result, status = await loop.create_task(action_client.send_goal())
    logger.info(f'A) result {result} and status flag {status}')
    result, status = await loop.create_task(action_client.send_goal())
    logger.info(f'B) result {result} and status flag {status}')

    # cancel spinning task
    spin_task.cancel()
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))


if __name__ == '__main__':
    main()