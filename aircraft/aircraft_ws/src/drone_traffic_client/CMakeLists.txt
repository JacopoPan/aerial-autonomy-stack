cmake_minimum_required(VERSION 3.8)
project(drone_traffic_client)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autopilot_interface_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(dtc_client src/dtc_client_node.cpp)

# Link the JSON library
target_link_libraries(dtc_client nlohmann_json::nlohmann_json)

ament_target_dependencies(dtc_client 
                          rclcpp 
                          rclcpp_action 
                          std_msgs 
                          autopilot_interface_msgs
                          )

install(TARGETS
  dtc_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
