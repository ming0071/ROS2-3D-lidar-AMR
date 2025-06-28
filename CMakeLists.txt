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
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# ========================================
# ⬇️ Build nodes
# ========================================

# 🔹 原本的單目標版本
add_executable(single_navigate_and_log_node src/single_navigate_and_log_node.cpp)
ament_target_dependencies(single_navigate_and_log_node
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
  tf2_ros
)

# 🔹 多目標點 YAML 版本
add_executable(multi_navigate_and_log_node src/multi_navigate_and_log_node.cpp)
ament_target_dependencies(multi_navigate_and_log_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

# ========================================
# ⬇️ Install executables
# ========================================
install(TARGETS
  single_navigate_and_log_node
  multi_navigate_and_log_node
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
