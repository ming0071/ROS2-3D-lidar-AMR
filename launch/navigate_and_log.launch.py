from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scl_amr',
            executable='goal_publisher',
            name='goal_publisher_node',
            parameters=[{
                'x': 6.4,
                'y': -0.4,
                'z': 0.0,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0
            }],
            output='screen'
        )
    ])
