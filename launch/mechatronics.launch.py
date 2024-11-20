import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # Include other launch files
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scl_amr"),
                "launch",
                "AMR_model.launch.py",
            ),
        )
    )
    hokuyo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("urg_node2"),
                "launch",
                "urg_node2_2lidar.launch.py",
            ),
        )
    )
    merged_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ira_laser_tools"),
            "launch",
            "laserscan_multi_merger.launch",
        ),
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("serial_imu"),
                "launch",
                "imu_pub.launch.py",
            ),
        )
    )
    micro_ros_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyACM0"],
        output="screen",  # This ensures output is visible in the terminal
    )

    return LaunchDescription(
        [
            tf_launch,
            hokuyo_launch,
            merged_launch,
            imu_launch,
            micro_ros_node,
        ]
    )
