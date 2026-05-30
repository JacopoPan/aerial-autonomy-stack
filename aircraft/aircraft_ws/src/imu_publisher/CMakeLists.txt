cmake_minimum_required(VERSION 3.8)
project(imu_publisher)

# ament dependencies
find_package(ament_cmake REQUIRED)

# ROS dependecies
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_publisher src/imu_publisher_node.cpp)

ament_target_dependencies(imu_publisher rclcpp sensor_msgs px4_msgs)

install(TARGETS
  imu_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
