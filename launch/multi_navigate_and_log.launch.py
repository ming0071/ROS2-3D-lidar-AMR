from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='scl_amr',
            executable='multi_navigate_and_log_node',
            name='multi_waypoint_navigator',
            parameters=[],
        ),

    ])