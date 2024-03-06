import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


from launch_ros.actions import Node

import xacro


def generate_launch_description():
    robot_name = "sence"
    hardware_package_name = robot_name + "_hardware"
    description_package_name = robot_name + "_description"
    controllers_package_name = robot_name + "_controllers"

    rviz_config = os.path.join(get_package_share_directory(
        description_package_name), "rviz", robot_name + ".rviz")

    robot_description_path = os.path.join(get_package_share_directory(
        hardware_package_name), "urdf", robot_name + "_hardware.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description_path,
                                                  mappings={'use_dummy': 'true'}
                                                  ).toxml()
    robot_description = {'robot_description': robot_description_config}

    controller_config = os.path.join(
        get_package_share_directory(
            controllers_package_name), "config", "controllers.yaml"
    )

    return LaunchDescription([

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description, controller_config],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                robot_description],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),

        Node(
            package="sence_poser",
            executable="sequence_action_server",
            name="sequence_action_server",
            output="screen"
        ),

        Node(
            package="sence_poser",
            executable="command_sub",
            name="command_sub",
            output="screen"
        )
    ])