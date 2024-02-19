import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool

from .sence_poses import sequences


class SenceCommandMenu(Node):
    def __init__(self):
        super().__init__('sence_action_menu')

        self.publisher_ = self.create_publisher(String, 'sence_commands', 10)

        self.loop_toggle_client = self.create_client(SetBool, 'set_looping')
        self.req = SetBool.Request()

        while True:
            print("""Sence Action Menu
1. Run Pose/Sequence
2. Run Loop
3. Stop Looping
q. Quit""")
            try:
                choice = input(">> ")
                match choice:
                    case '1':
                        self.setLooping(False)
                        self.poseMenu()
                    case '2':
                        self.setLooping(True)
                        self.poseMenu()
                    case '3':
                        self.setLooping(False)
                    case 'q':
                        self.setLooping(False)
                        break
                    case _:
                        print("Not an option")
            except Exception as e:
                print(e)

    def setLooping(self, state):
        self.req.data = state
        self.loop_toggle_client.call_async(self.req)

    def poseMenu(self):
        enumerated_poses = list(enumerate(list(sequences.keys())))
        while True:
            for (index, pose) in enumerated_poses:
                print(f"{index}. {pose}")
            print("Enter a pose (q to return)")

            try:
                response = input(">> ")
                if response == "q":
                    break

                selected_pose = enumerated_poses[int(response)][1]

                msg = String()
                msg.data = selected_pose
                self.publisher_.publish(msg)

            except Exception as e:
                print(e)

def main(args=None):
    rclpy.init(args=args)

    action_menu = SenceCommandMenu()

    action_menu.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()