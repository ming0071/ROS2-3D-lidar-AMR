<div align="center">
<h1>Autonomous Mobile Robot (AMR) Based on ROS 2</h1>
</div>

本專案提出一套基於 ROS 2 Jazzy 架構的自主移動機器人（Autonomous Mobile Robot, AMR）系統，整合 3D LiDAR 與 IMU 進行地圖建構、定位、路徑規劃與運動控制，並透過 2D LiDAR 實現動態障礙物偵測與避障，應用於室內導航任務。

在定位方面，系統採用 DLIO（LiDAR-Inertial Odometry）進行高頻姿態估測，並結合 AMCL 以補償累積誤差。在路徑規劃與控制方面，分別比較了 A* 與 Theta* 演算法，以及 DWB 與 MPPI 控制器。最終選用 Theta* 搭配 MPPI，實驗顯示可實現更平滑的軌跡與穩健的避障控制能力。

<img src="doc/figure/AMR system structure.png" />

系統於兩組不同導航任務中進行驗證，包含 38 公尺動態障礙路徑與 185 公尺長距離走廊導航。結果顯示平均位置誤差小於 5.1 公分，角度誤差小於 3.5 度，且移動軌跡具備良好的一致性。Gif 動圖中演示 AMR 於動態障礙路徑的移動過程：

<img src="doc/figure/A point.gif" />

## Hardware Configuration

- 3D LiDAR: Velodyne VLP-16 Puck
- 2D LiDARs: Hokuyo x2 (diagonally mounted for 360° scanning)
- IMU: CH110
- Microcontroller: OpenCR (with micro-ROS support)

> 詳細的設備規格請參閱[SCL AMR 硬體系統](https://hackmd.io/@ming0071/scl-amr)，設備連接設定請參閱[ROS2 Hokuyo、Velodyne、IMU 設定](https://hackmd.io/@ming0071/hokuyo-velodyne-imu-setup)

## Software Architecture

### Using packages

- Mechatronic System:
    - [velodyne](https://github.com/ros-drivers/velodyne/tree/ros2)
    - [urg_node2](https://github.com/Hokuyo-aut/urg_node2)
    - [ira_laser_tools](https://github.com/nakai-omer/ira_laser_tools/tree/humble)
    - [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/jazzy)
    - [micro-ROS-agent](https://github.com/micro-ROS/micro-ROS-Agent/tree/jazzy)
    - [micro_ros_msgs](https://github.com/micro-ROS/micro_ros_msgs/tree/jazzy)
    - [serial_imu](https://sealandtech.com.tw/resource.html?s=anrot&type=tutorial&p=ros2/readme)
- Mapping:
    - [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/tree/feature/ros2)
    - pcl_viewer: ```apt-get install pcl-tools```
- Localization: AMCL + DLIO odometry
    - AMCL: from navigation2, alpha(1~5) = 0.01
    - [DLIO](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/tree/feature/ros2)
- Navigation2:
    - 2d map: [pointcloud_to_2dmap_ros](https://github.com/m11312045/pointcloud_to_2dmap_ros/tree/ros2)
    - global planner: Theta* path planner
    - local planner: MPPI (Model Predictive Path Integral) controller

> 詳細的 ROS 2 與 package 安裝指南請參閱[ROS2 Jazzy Package 安裝：機電系統、建圖、導航](https://hackmd.io/@ming0071/ROS2-Jazzy-install)

### Chassis control

1. OpenCR reads encoder signals from both wheels.
2. Receives /cmd_vel from the ROS 2 navigation stack.
3. Converts central velocity commands into left/right wheel speeds using differential drive kinematics (not Ackermann).
4. Applies PID control for velocity tracking and sends PWM signals to the motors.

> 詳細的說明請參閱[Micro-ROS 配合 ESP32、OpenCR 安裝與設定](https://hackmd.io/@ming0071/micro-ros-install-setup)、[ROS2 AMR OpenCR底盤驅動](https://hackmd.io/@ming0071/opencr-chassic-firmware)

## Launch command

> 詳細的指令請參閱[ROS2 launch 指令集](https://hackmd.io/@ming0071/ros2-launch-commands)，此處僅列出基本使用指令

### Mechatronics system

#### main (sencor + tf + chassic)
```bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch scl_amr mechatronics.launch.py
```

### Off-Line SLAM

#### Record bag file
```bash
cd ~/ros2_ws/src/scl_amr/bagfile/
ros2 bag record -a -o b1_2rounds_no_vehivle
```

#### DLIO mapping
```bash
# bag 
cd ~/ros2_ws/src/scl_amr/bagfile/
ros2 bag play b1_2rounds_no_vehivle

# slam
cd ~/ros2_ws/
source install/setup.bash
ros2 launch scl_amr dlio.launch.py

# save pcd
cd ~/ros2_ws/
source install/setup.bash
ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD "{'leaf_size': 0.1, 
'save_path': '/home/scl/ros2_ws/src/scl_amr/map/'}"

# pcd view
pcl_viewer ~/ros2_ws/src/scl_amr/map/dlio_map.pcd

# pointcloud_to_2dmap_ros2
ros2 launch pointcloud_to_2dmap_ros2 pointcloud_to_2dmap.launch.py
```

### Navigation

#### Nav2 
```bash
# mechatronics 
cd ~/ros2_ws/
source install/setup.bash
ros2 launch scl_amr mechatronics.launch.py

# odometry
cd ~/ros2_ws/
source install/setup.bash
ros2 launch scl_amr dlio.launch.py

# navigation
cd ~/nav2_ws/
source install/setup.bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch scl_amr navigation_bringup_launch.py

# 1. single navigate
cd ~/nav2_ws/
source install/setup.bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch scl_amr single_navigate_and_log.launch.py mode:=1

# 2. multi navigate
cd ~/nav2_ws/
source install/setup.bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch scl_amr multi_navigate_and_log.launch.py log_csv:=5.csv
```

#### rviz update
```bash
cp /home/scl/ros2_ws/install/scl_amr/share/scl_amr/rviz/nav2_scl.rviz /home/scl/ros2_ws/src/scl_amr/rviz/
```

#### About tf
```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover

ros2 run tf2_ros tf2_echo map odom 
```