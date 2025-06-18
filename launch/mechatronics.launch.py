import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # Include other launch files
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

    # micro-ROS Node
    micro_ros_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyACM0"],
        output="screen",
    )
    text = "[Hey Bro] Press the reset button on the OpenCR to start the chassis program !"
    final_message = TimerAction(
        period=5.0,     # float
        actions=[LogInfo(msg="\033[1;34m" + text + "\033[0m")],
    )

    return LaunchDescription(
        [
            sensor_launch,
            micro_ros_node,
            final_message,
        ]
    )
