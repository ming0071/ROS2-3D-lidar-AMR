cmake_minimum_required(VERSION 3.8)
project(scl_amr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ========================================
# ⬇️ Find ROS 2 dependencies
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ========================================
# ⬇️ Build mine node
# ========================================
add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher_and_log
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
  tf2_ros
)

# ========================================
# ⬇️ Install executables
# ========================================
install(TARGETS
  goal_publisher
  pose_logger
  DESTINATION lib/${PROJECT_NAME}
)

# ========================================
# ⬇️ Install all your existing resources
# ========================================

foreach(dir launch config urdf meshes rviz data map)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir}
  )
endforeach()

ament_package()
