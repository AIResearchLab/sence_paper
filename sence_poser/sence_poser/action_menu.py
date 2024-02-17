import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus

from sence_msgs.action import StaticPose, PoseSequence

from std_msgs.msg import String

from .sence_poses import poses, sequences, loops


class SenceActionMenu(Node):
    def __init__(self):
        super().__init__('sence_action_menu')

        self.publisher_ = self.create_publisher(String, 'sence_commands', 10)

        while True:
            print("""Sence Action Menu
0. Pose Client
1. Sequence Client
2. Loop Client
3. append standup to schedule
q. Quit""")
            try:
                choice = input(">> ")
                match choice:
                    case '0':
                        self.poseMenu()
                    case '1':
                        self.sequenceMenu()
                    case '2':
                        self.loopMenu()
                    case '3':
                        msg = String()
                        msg.data = "crab_stand_up"
                        self.publisher_.publish(msg)
                        self.get_logger().info('Publishing: "%s"' % msg.data)
                    case 'q':
                        break
                    case _:
                        print("Not an option")
            except Exception as e:
                print(e)


    def poseMenu(self):
        self.pose_action_client = ActionClient(
            self,
            StaticPose,
            '/static_pose')

        self.get_logger().info('Waiting for pose server...')
        self.pose_action_client.wait_for_server()

        enumerated_poses = list(enumerate(list(poses.keys())))
        while True:
            for (index, pose) in enumerated_poses:
                print(f"{index}. {pose}")
            print("Enter a pose (q to return)")

            try:
                response = input(">> ")
                if response == "q":
                    break                
                selected_pose = enumerated_poses[int(response)][1]

                goal_msg = StaticPose.Goal()
                goal_msg.pose = selected_pose

                self.get_logger().info('Sending goal request...')
                self.pose_action_client.send_goal_async(goal_msg)

            except Exception as e:
                print(e)


    def sequenceMenu(self):
        self.sequence_action_client = ActionClient(
            self,
            PoseSequence,
            '/pose_sequence')
        
        self.get_logger().info('Waiting for sequence server...')
        self.sequence_action_client.wait_for_server()

        enumerated_sequences = list(enumerate(list(sequences.keys())))

        while True:
            for (index, sequence) in enumerated_sequences:
                print(f"{index}. {sequence}")
            print("Enter a sequence (q to return)")

            try:
                response = input(">> ")
                if response == "q":
                    break

                selected_sequence = enumerated_sequences[int(response)][1]

                goal_msg = PoseSequence.Goal()
                goal_msg.sequence = selected_sequence

                self.get_logger().info('Sending goal request...')
                self.sequence_action_client.send_goal_async(goal_msg)

            except Exception as e:
                print(e)

    async def loopMenu(self):
        self.loop_action_client = ActionClient(
            self,
            PoseSequence,
            '/sequence_loop')
        
        self.get_logger().info('Waiting for loop server...')
        self.loop_action_client.wait_for_server()

        enumerated_loops = list(enumerate(list(loops)))

        while True:
            for (index, sequence) in enumerated_loops:
                print(f"{index}. {sequence}")
            print("Enter a loop (q to return)")

            try:
                response = input(">> ")
                if response == "q":
                    break

                selected_sequence = enumerated_loops[int(response)][1]

                goal_msg = PoseSequence.Goal()
                goal_msg.sequence = selected_sequence

                self.get_logger().info('Sending goal request...')
                goal_handle = await self.loop_action_client.send_goal_async(goal_msg)
            
                self.get_logger().info('Goal accepted :)')

                res = await goal_handle.get_result_async()
                result = res.result
                status = res.status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info('Goal succeeded! Result: {0}'.format(result.error_code))
                else:
                    self.get_logger().info('Goal failed with status: {0}'.format(status))

            except Exception as e:
                print(e)


def main(args=None):
    rclpy.init(args=args)

    action_menu = SenceActionMenu()

    # rclpy.spin(action_menu)

    action_menu.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()