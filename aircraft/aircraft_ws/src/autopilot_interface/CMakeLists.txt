cmake_minimum_required(VERSION 3.8)
project(autopilot_interface)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autopilot_interface_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add the CMake module path
set(CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib" ${CMAKE_MODULE_PATH})
find_package(GeographicLib REQUIRED)

add_executable(px4_interface src/px4_interface.cpp)
target_link_libraries(px4_interface ${GeographicLib_LIBRARIES})
ament_target_dependencies(px4_interface 
                          rclcpp
                          rclcpp_action
                          px4_msgs
                          geometry_msgs
                          autopilot_interface_msgs
                          std_msgs
                          )

add_executable(ardupilot_interface src/ardupilot_interface.cpp)
target_link_libraries(ardupilot_interface ${GeographicLib_LIBRARIES})
ament_target_dependencies(ardupilot_interface
                          rclcpp
                          rclcpp_action
                          mavros_msgs
                          geometry_msgs
                          nav_msgs
                          sensor_msgs
                          autopilot_interface_msgs
                          geographic_msgs
                          std_msgs
                          )

install(TARGETS
  px4_interface
  ardupilot_interface
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
