cmake_minimum_required(VERSION 3.8)
project(px4_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(px4_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    px4_common
    rclcpp
    geometry_msgs
    nav_msgs
    trajectory_msgs
)

ament_package()
