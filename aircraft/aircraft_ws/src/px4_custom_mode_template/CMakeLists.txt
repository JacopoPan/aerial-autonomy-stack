cmake_minimum_required(VERSION 3.5)
project(px4_custom_mode_template)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})
add_executable(px4_custom_mode_template
        src/main.cpp)
target_link_libraries(px4_custom_mode_template PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp)

install(TARGETS
        px4_custom_mode_template
        DESTINATION lib/${PROJECT_NAME})

ament_package()
