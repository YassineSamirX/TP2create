"""
Simulation Launch File
========================
Launches Gazebo with Create3 + the full mission system.
Includes the Create3 Gazebo simulation and all mission nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    patrol_duration_arg = DeclareLaunchArgument(
        'patrol_duration', default_value='120.0',
        description='Duration of patrol phase in seconds')

    world_arg = DeclareLaunchArgument(
        'world', default_value='',
        description='Gazebo world file')

    # ── Gazebo ──
    # NOTE: Adjust this to your Create3 simulation setup
    # For irobot_create3_gazebo, use:
    #   IncludeLaunchDescription(
    #       PythonLaunchDescriptionSource([
    #           get_package_share_directory('irobot_create_gazebo_bringup'),
    #           '/launch/create3_gazebo.launch.py']),
    #       launch_arguments={'world': LaunchConfiguration('world')}.items(),
    #   )
    # For a minimal setup without the full irobot package:
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    # ── Dock Status Publisher (simulation mock) ──
    # Publishes /is_docked = true initially, then false after undock
    dock_status_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/is_docked', 'std_msgs/msg/Bool', '{data: true}',
        ],
        output='screen',
    )

    # ── BT Executor ──
    bt_executor = Node(
        package='create3_bt',
        executable='bt_executor_node',
        name='bt_executor',
        output='screen',
        parameters=[{
            'tick_rate': 10.0,
            'patrol_duration': LaunchConfiguration('patrol_duration'),
            'waypoints': [1.0, 0.0,  1.0, 1.0,  0.0, 1.0],
        }],
    )

    # ── Draw Square Server (Assignment 1) ──
    draw_square = Node(
        package='draw_square',
        executable='draw_square_server',
        name='draw_square_server',
        output='screen',
    )

    # ── Boundary Follow Server (Assignment 2) ──
    boundary_follow = Node(
        package='boundary_follow',
        executable='boundary_follow_server',
        name='boundary_follow_server',
        output='screen',
    )

    # ── Teleop Override Bridge ──
    override_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'create3_bt', 'teleop_override_bridge'],
        name='teleop_override_bridge',
        output='screen',
        prefix='xterm -e',
    )

    # ── Teleop Keyboard ──
    teleop = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '-r', 'cmd_vel:=/cmd_vel_teleop',
        ],
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([
        patrol_duration_arg,
        world_arg,
        gazebo,
        dock_status_pub,
        bt_executor,
        draw_square,
        boundary_follow,
        override_bridge,
        teleop,
    ])
