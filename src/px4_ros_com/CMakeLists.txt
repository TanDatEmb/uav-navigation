cmake_minimum_required(VERSION 3.8)
project(px4_ros_com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_common REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    px4_common
    px4_msgs
    rclcpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
)

ament_package()
