from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('log_csv', default_value='5.csv'), 

        Node(
            package='scl_amr',
            executable='multi_navigate_and_log_node',
            name='multi_waypoint_navigator',
            parameters=[{
            'log_csv': LaunchConfiguration('log_csv')
            }],
        ),

    ])