
import asyncio
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .sence_poses import sequences


class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')

        self.schedule = []

        self._command_sub = self.create_subscription(String, "sence_commands",  self.command_callback, 10);

    def command_callback(self, msg):
        self.get_logger().info(f'appending command {msg}')
        self.schedule.append(msg.data)

    def doNextAction(self):
        if len(self.schedule) > 0:
            next_action = self.schedule.pop(0)

            try:
                desired_sequence = sequences[next_action]

                for pose in desired_sequence:
                    self.get_logger().info(f'moving to pose {pose}')

            except Exception as e:
                self.get_logger().info('there was a problem with the last action')
                self.get_logger().info(str(e))

        else:
            self.get_logger().info(f'nothing to do...')
            # self.get_logger().info(str(self.schedule))


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
                       
            node.doNextAction()

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