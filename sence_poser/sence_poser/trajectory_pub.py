import rclpy
from rclpy.node import Node
from .sence_poses import *

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.poseTime = 1

    def poseMenu(self):
        while (True):
            try:
                print("""sence trajectory publisher
    1: flat
    2: curl
    3: crab stand
    4: dog ready
    5: dog flat
    6: dog stand
    7: dog walk
    9: set time to pose (currently {ttp}sec)
    0: quit""".format(ttp=self.poseTime)
                )
                response = int(input("enter a number: "))
                match response:
                    case 1:
                        self.publishPose(flatPose)
                    case 2:
                        self.publishPose(curlPose)
                    case 3:
                        self.publishPose(crabStandPose)
                    case 4:
                        self.publishPose(dogReadyPose)
                    case 5:
                        self.publishPose(dogFlatPose)
                    case 6:
                        self.publishPose(dogStandPose)
                    case 7:
                        step = 0
                        while (True):
                            try:
                                for stepPose in dogWalkPoses:
                                    self.publishPose(stepPose)
                                    print(f"step {step}")
                                    step += 1
                                    input()
                            except KeyboardInterrupt:
                                pass
                    case 9:
                        self.changeSpeed()
                    case 0:
                        break;
                    case _:
                        print("unknown response")

            except Exception as e:
                print(e)

    def publishPose(self, goal):
        msg = JointTrajectory()
        msg.joint_names = jointNames
        point1 = JointTrajectoryPoint()
        point1.positions = goal
        point1.time_from_start.sec = self.poseTime
        msg.points.append(point1)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing now')

    def changeSpeed(self):
        try:
            self.poseTime = int(input("enter value in seconds: "))
        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = TrajectoryPublisher()
    #rclpy.spin(trajectory_publisher)

    trajectory_publisher.poseMenu()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
