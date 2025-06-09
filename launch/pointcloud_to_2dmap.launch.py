from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    map_name = "0.5d" 

    return LaunchDescription([
        Node(
            package='pointcloud_to_2dmap_ros2',
            executable='pointcloud_to_2dmap_node',
            name='pointcloud_to_2dmap_node',
            output='screen',
            parameters=[
                {"resolution": 0.1},
                {"map_width": 1300},
                {"map_height": 700},
                {"min_points_in_pix": 2},
                {"max_points_in_pix": 7},
                {"min_height": 0.5},
                {"max_height": 2.0},    # max hight of floor: 2.5
                {"dest_directory": "/home/scl/ros2_ws/src/scl_amr/map"},
                {"input_pcd": f"/home/scl/ros2_ws/src/scl_amr/map/{map_name}.pcd"},
                {"map_name": map_name}
            ]
        )
    ])
