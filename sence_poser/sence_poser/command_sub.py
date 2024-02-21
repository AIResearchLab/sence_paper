
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

from std_srvs.srv import SetBool, Trigger

from .sence_poses import jointNames, sequences, poses, poseSec, poseNano


class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')

        self.schedule = []

        self._command_sub = self.create_subscription(String, "sence_commands",  self.command_callback, 10);

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.isLooping = False
        self._set_looping_service = self.create_service(SetBool, 'set_looping', self.set_looping_callback)

        self.isStepping = False
        self._toggle_stepping_service = self.create_service(Trigger, 'toggle_stepping', self.toggle_stepping_callback)

    def command_callback(self, msg):
        self.get_logger().info(f'appending command {msg}')
        self.schedule.append(msg.data)

    def set_looping_callback(self, request, response):
        self.get_logger().info('setting loop state to ' + str(request.data))
        
        response = SetBool.Response()

        if request.data != self.isLooping:
            self.isLooping = request.data
            response.success = True
        else:
            response.success = False

        return response
    
    def toggle_stepping_callback(self, request, response):
        response = Trigger.Response()

        if self.isStepping:
            self.get_logger().info('disabling stepping')
            self.isStepping = False
        else:
            self.get_logger().info('enabling stepping')
            self.isStepping = True
        
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
    cmd_sub = CommandInterface()
    logger = rclpy.logging.get_logger('commandy_subby')
    logger.info('sence command sub running...')

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(cmd_sub, ), daemon=True)
    thread.start()

    rate = cmd_sub.create_rate(2)

    try:
        while rclpy.ok():
            
            if len(cmd_sub.schedule) > 0:
                if cmd_sub.isStepping:
                    input("press enter to run next action")

                next_action = cmd_sub.schedule.pop(0)

                if next_action in sequences:

                    logger.info(f'running sequence {next_action}')

                    result, status = await loop.create_task(cmd_sub.send_goal(next_action))
                    logger.info(f'A) result {result} and status flag {status}')

                    while cmd_sub.isLooping:
                        logger.info(f'looping on {next_action}')

                        if cmd_sub.isStepping:
                            input("press enter to run next action")

                        result, status = await loop.create_task(cmd_sub.send_goal(next_action))
                        logger.info(f'A) result {result} and status flag {status}')


                else:
                    logger.info(f'sequence not found {next_action}')

            # else:
            #     logger.info(f'nothing to do...')
                
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