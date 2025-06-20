# ROS2 3D lidar AMR develop

I have a [detailed article](https://hackmd.io/@ming0071/ROS2-notebook/%2F%40ming0071%2Fubuntu-reinstall-software) introducing this project.

## Sensor 
- Velodyne VLP-16 Puck 3D LiDAR
- Hokuyo *2
- IMU CH110

## Chassis control

1. Use the OpenCR controller to read encoder signals.
2. Receive the cmd_vel topic from ROS, and convert the central velocity into left and right wheel speeds using Ackermann steering.
3. Apply PID control to calculate the velocity commands and send PWM signals to the motors.

## Algorithm package

- Mechatronic System:
    - velodyne
    - urg_node2
    - ira_laser_tools
    - micro_ros_arduino
    - micro_ros_agent
    - micro_ros_msgs
    - serial_imu
- Mapping:
    - DLIO
- Localization:
    - AMCL
    - DLIO
- Navigation2:
    - 2d map : [pointcloud_to_2dmap_ros](https://github.com/m11312045/pointcloud_to_2dmap_ros/tree/ros2)
    - local planner : MPPI
    - global planner : theta*