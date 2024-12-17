import os

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # sensor
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scl_amr"),
                "launch",
                "sensor",
                "sensor.launch.py",
            ),
        )
    )
    # micro ros
    micro_ros_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyACM0"],
        output="screen",  # This ensures output is visible in the terminal
    )
    # faster lio2
    faster_lio_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("scl_amr"),
            "launch",
            "offline_faster_lio.launch.py",
        ),
    )

    return LaunchDescription(
        [
            sensor_launch,
            micro_ros_node,
            faster_lio_launch,
        ]
    )
