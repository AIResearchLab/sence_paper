
import asyncio
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sence_msgs.srv import Trigger

from .sence_poses import jointNames, sequences, poses, poseSec, poseNano


class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')

        self.schedule = []
        self.isLooping = False

        self._command_sub = self.create_subscription(String, "sence_commands",  self.command_callback, 10);

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        
        self._loop_switch_service = self.create_service(Trigger, 'loop_toggle', self.loop_toggle_callback)


    def command_callback(self, msg):
        self.get_logger().info(f'appending command {msg}')
        self.schedule.append(msg.data)

    def loop_toggle_callback(self, request, response):
        if self.isLooping:
            self.isLooping = False
        else:
            self.isLooping = True
        return response

    def build_goal_msg(self, goal_sequence):
        # # Create the goal message
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
        
        return goal_msg


    async def send_goal(self, command):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = self.build_goal_msg(command)

        self.get_logger().info('Sending goal request...')

        goal_handle = await self._action_client.send_goal_async(
            goal_msg,
            # feedback_callback=self.feedback_callback
        )

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return 'failed', 'failed'

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
    rclpy.init()
    node = CommandInterface()
    logger = rclpy.logging.get_logger('commandy_subby')

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    rate = node.create_rate(2)

    try:
        while rclpy.ok():
            
            if len(node.schedule) > 0:
                next_action = node.schedule.pop(0)

                if next_action in sequences:

                    logger.info(f'running sequence {next_action}')

                    result, status = await loop.create_task(node.send_goal(next_action))
                    logger.info(f'A) result {result} and status flag {status}')

                    while node.isLooping:
                        logger.info(f'looping on {next_action}')

                        result, status = await loop.create_task(node.send_goal(next_action))
                        logger.info(f'A) result {result} and status flag {status}')


                else:
                    logger.info(f'sequence not found {next_action}')

            else:
                logger.info(f'nothing to do...')
                # self.get_logger().info(str(self.schedule))
                
            rate.sleep()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()

def main(args=None):
    loop = asyncio.get_event_loop()

    loop.run_until_complete(run(args, loop=loop))

if __name__ == '__main__':
    main()