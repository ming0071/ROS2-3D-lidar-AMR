import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():

    version = "ROS2"

    if version == "ROS2":
        urdf_name = "AMR_model.urdf"  # AMR_model.urdf    AMR_ros_3d.urdf.xacro
    else:
        urdf_name = "AMR_ros_3d.urdf.xacro"
    urdf_model_path = os.path.join(
        get_package_share_directory("scl_amr"),
        "urdf",
        urdf_name,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_model_path],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[urdf_model_path],
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    return ld
