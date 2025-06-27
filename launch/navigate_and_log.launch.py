from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # 可被外部指定的參數宣告
        DeclareLaunchArgument('mode', default_value='0'),    # 0:start , 1: goal
        DeclareLaunchArgument('log_csv', default_value='goal.csv'), 

        Node(
            package='scl_amr',
            executable='goal_publisher_and_log',
            name='goal_publisher_node',
            parameters=[{
                'mode': LaunchConfiguration('mode'),     
                
                "start.x": -6.29,
                'start.y': -2.75,
                'start.z': 0.0,
                'start.qx': 0.0,
                'start.qy': 0.0,
                'start.qz': 1.0,
                'start.qw': 0.0,
                
                'goal.x': 2.17,
                'goal.y': -2.54,
                'goal.z': 0.0,
                'goal.qx': 0.0,
                'goal.qy': 0.0,
                'goal.qz': 0.0,
                'goal.qw': 1.0,

                'log_csv': LaunchConfiguration('log_csv')
            }],
            output='screen'
        )
    ])