import os

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # urdf
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scl_amr"),
                "launch",
                "AMR_model.launch.py",
            ),
        )
    )
    # urg node 2
    hokuyo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scl_amr"),
                "launch",
                "sensor",
                "hokuyo.launch.py",
            ),
        )
    )
    # ira laser tools
    merged_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("scl_amr"),
            "launch",
            "sensor",
            "laser_merger.launch",
        ),
    )
    # velodyne
    VLP16_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scl_amr"),
                "launch",
                "sensor",
                "velodyne.launch.py",
            ),
        )
    )
    # serial imu
    imu_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("scl_amr"),
            "launch",
            "sensor",
            "imu.launch.py",
        ),
    )

    return LaunchDescription(
        [
            tf_launch,
            hokuyo_launch,
            merged_launch,
            VLP16_launch,
            imu_launch,
        ]
    )
