from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    speed_arg = DeclareLaunchArgument(
        'speed', default_value='2.0',
        description='Maximum linear/angular speed of the turtle')

    boundary_margin_arg = DeclareLaunchArgument(
        'boundary_margin', default_value='0.5',
        description='Distance from the turtlesim wall edge defining the boundary')

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen',
    )

    draw_boundaries_node = Node(
        package='turtle_boundary_controller',
        executable='draw_boundaries_node',
        name='draw_boundaries_node',
        output='screen',
        parameters=[{
            'speed': LaunchConfiguration('speed'),
            'boundary_margin': LaunchConfiguration('boundary_margin'),
        }],
    )

    # Keyboard listener needs its own terminal for raw input
    keyboard_listener_node = ExecuteProcess(
        cmd=['ros2', 'run', 'turtle_boundary_controller', 'keyboard_listener'],
        name='keyboard_listener',
        output='screen',
        prefix='xterm -e',
    )

    # Turtlesim teleop for manual mode — needs its own terminal too
    teleop_node = ExecuteProcess(
        cmd=['ros2', 'run', 'turtlesim', 'turtle_teleop_key'],
        name='turtle_teleop_key',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([
        speed_arg,
        boundary_margin_arg,
        turtlesim_node,
        draw_boundaries_node,
        keyboard_listener_node,
        teleop_node,
    ])
