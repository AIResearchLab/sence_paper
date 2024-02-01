import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_joint_trajectory_goal():
    rclpy.init()

    try:
        node = rclpy.create_node('joint_trajectory_publisher')

        # Create an action client for FollowJointTrajectory action
        action_client = ActionClient(node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        # Wait for the action server to be available
        print("Waiting for action server...")
        action_client.wait_for_server()
        print("ok")

        # Create the FollowJointTrajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['fl_joint1']

        # Define trajectory points
        points = [
            {'positions': [1.0], 'time_from_start': Duration(sec=2)},
            {'positions': [-1.0], 'time_from_start': Duration(sec=4)},
            {'positions': [0.0], 'time_from_start': Duration(sec=6)},
        ]

        for point_data in points:
            point_msg = JointTrajectoryPoint()
            point_msg.positions = point_data['positions']
            point_msg.time_from_start = point_data['time_from_start']
            goal_msg.trajectory.points.append(point_msg)

        # Send the goal and wait for the result
        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(node, future)

        result = future.result()
        if result:
            node.get_logger().info('Goal succeeded!')
        else:
            node.get_logger().warning('Goal failed!')

    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    send_joint_trajectory_goal()

if __name__ == '__main__':
    main()
