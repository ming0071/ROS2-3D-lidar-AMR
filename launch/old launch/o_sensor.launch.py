import os

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # Include launch file
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
    VLP16_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("velodyne"),
                "launch",
                "velodyne-all-nodes-VLP16-launch.py",
            ),
        )
    )
    imu_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("serial_imu"),
            "launch",
            "imu_pub.launch.py",
        ),
    )

    ld = LaunchDescription()
    ld.add_action(tf_launch)
    ld.add_action(hokuyo_launch)
    ld.add_action(merged_launch)
    ld.add_action(VLP16_launch)
    ld.add_action(imu_launch)

    return ld
