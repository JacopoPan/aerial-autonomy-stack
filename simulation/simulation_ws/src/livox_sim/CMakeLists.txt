cmake_minimum_required(VERSION 3.8)
project(livox_sim)

# ament dependencies
find_package(ament_cmake REQUIRED)

# ROS dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Gazebo dependencies
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(livox_sim src/livox_sim_node.cpp)

ament_target_dependencies(livox_sim rclcpp sensor_msgs livox_ros_driver2)
target_link_libraries(livox_sim gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)

install(TARGETS
  livox_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
