from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 可被外部指定的參數宣告
        DeclareLaunchArgument('mode', default_value='0'),    # 0:start , 1: goal
        DeclareLaunchArgument('log_csv', default_value='3.csv'), 

        Node(
            package='scl_amr',
            executable='single_navigate_and_log_node',
            name='goal_publisher_node',
            parameters=[{
                'mode': LaunchConfiguration('mode'),     
                
                "start.x": -6.23,
                'start.y': -2.89,
                'start.z': 0.0,
                'start.qx': 0.0,
                'start.qy': 0.0,
                'start.qz': 1.0,
                'start.qw': 0.0,
                
                'goal.x': -6.23,
                'goal.y': 8.49,
                'goal.z': 0.0,
                'goal.qx': 0.0,
                'goal.qy': 0.0,
                'goal.qz': 1.0,
                'goal.qw': 0.0,

                'log_csv': LaunchConfiguration('log_csv')
            }],
            output='screen'
        )
    ])