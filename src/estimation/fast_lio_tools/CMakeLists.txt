cmake_minimum_required(VERSION 3.16)
project(fast_lio_tools LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(fast_lio_core REQUIRED)
find_package(fast_lio_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(lio_offline src/lio_offline.cpp)
target_compile_options(lio_offline PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(lio_offline
  fast_lio_core fast_lio_ros livox_ros_driver2 rclcpp rosbag2_cpp sensor_msgs)

install(TARGETS lio_offline
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
