cmake_minimum_required(VERSION 3.8)
project(ground_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ground_system_msgs REQUIRED)

add_executable(ground_system
  src/ground_system.cpp
)

target_include_directories(ground_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

ament_target_dependencies(ground_system
  rclcpp
  std_msgs
  ground_system_msgs
)

install(TARGETS ground_system
  DESTINATION lib/${PROJECT_NAME})

ament_package()
