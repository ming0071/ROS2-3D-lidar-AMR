from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scl_amr',
            executable='goal_publisher',
            name='goal_publisher_node',
            parameters=[{
                'goal_x': 6.4,
                'goal_y': -0.4,
                'goal_yaw': 0.0,
            }],
            output='screen'
        ),
        # Node(
        #     package='scl_amr',
        #     executable='pose_logger',
        #     name='pose_logger_node',
        #     output='screen'
        # )
    ])
