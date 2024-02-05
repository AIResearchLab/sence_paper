import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    robot_name = "sence"
    this_package_path = os.path.join(
        get_package_share_directory(robot_name + "_gazebo"))
    description_package_path = os.path.join(
        get_package_share_directory(robot_name + "_description"))
    worlds_path = os.path.join(this_package_path, 'worlds')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[worlds_path]
    )

    # Process Xacro
    sence_xacro_file = os.path.join(this_package_path,
                              'urdf',
                              'sence_gazebo.urdf.xacro')
    sencedoc = xacro.parse(open(sence_xacro_file))
    xacro.process_doc(sencedoc)
    docxml = sencedoc.toxml()

    brick_urdf_file = os.path.join(this_package_path,
                              'urdf',
                              'brick.urdf')
    brickdoc = xacro.parse(open(brick_urdf_file))
    xacro.process_doc(brickdoc)
    brickxml = brickdoc.toxml()


    params = {'robot_description': docxml, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch gazebo environment
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'),
                            'launch', 'gz_sim.launch.py')]),
        launch_arguments=[(
            'gz_args', '-r empty1.sdf'
        )]
    )

    spawn_brick = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', brickxml,
                   '-name', "brick",
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.0']
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', docxml,
                   '-name', robot_name,
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.2']
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(description_package_path, "rviz/sence.rviz"),
        ],
        output="screen",
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    sequence_server = Node(
        package="sence_poser",
        executable="sequence_action_server",
        name="sequence_action_server",
        output="screen"
    )
    
    return LaunchDescription([
        ign_resource_path,
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),

        launch_gazebo,
        node_robot_state_publisher,
        spawn_brick,
        spawn_robot,
        bridge,
        sequence_server,
        # rviz,

        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])

