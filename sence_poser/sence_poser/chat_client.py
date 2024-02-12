import rclpy
from rclpy.action import ActionClient
from sence_msgs.action import PoseSequence
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class YourActionClient:
    def __init__(self):
        self.node = rclpy.create_node('your_action_client_node')
        # self.action_client = ActionClient(self.node, PoseSequence, '/pose_sequence')
        self.action_client = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')


    def send_goal(self):
        # goal_msg = PoseSequence.Goal()  # Customize this line according to your action goal definition
        # goal_msg.sequence = 'crab_dance'

        # Create the goal message
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

        self.goal_handle = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback if needed
        # This function will be called whenever the action server sends feedback
        pass

    def wait_for_goal_completion(self):
        return self.goal_handle.result()

    def start_action_loop(self):
        while rclpy.ok():
            print('sending...')
            self.send_goal()
            print('waiting...')
            self.wait_for_goal_completion()

def main():
    rclpy.init()
    action_client = YourActionClient()
    action_client.start_action_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()