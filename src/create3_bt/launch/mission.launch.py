"""
Mission Launch File
====================
Launches the full Create3 BT mission system:
  - bt_executor_node (BT engine)
  - draw_square_server (Assignment 1)
  - boundary_follow_server (Assignment 2)
  - teleop_override_bridge (manual override)
  - teleop_twist_keyboard (for manual control)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    patrol_duration_arg = DeclareLaunchArgument(
        'patrol_duration', default_value='120.0',
        description='Duration of patrol phase in seconds')

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
        parameters=[{
            'linear_speed': 0.15,
            'angular_speed': 0.5,
        }],
    )

    # ── Boundary Follow Server (Assignment 2) ──
    boundary_follow = Node(
        package='boundary_follow',
        executable='boundary_follow_server',
        name='boundary_follow_server',
        output='screen',
        parameters=[{
            'linear_speed': 0.12,
            'angular_speed': 0.6,
            'wall_distance': 0.35,
            'front_threshold': 0.40,
        }],
    )

    # ── Teleop Override Bridge ──
    override_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'create3_bt', 'teleop_override_bridge'],
        name='teleop_override_bridge',
        output='screen',
        prefix='xterm -e',
    )

    # ── Teleop Twist Keyboard (for manual control) ──
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
        bt_executor,
        draw_square,
        boundary_follow,
        override_bridge,
        teleop,
    ])
