cmake_minimum_required(VERSION 3.22)
project(uavnav_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

# No ROS (rclcpp) dependency: this library must stay usable outside any node.
# INTERFACE for now; it becomes STATIC when the first source file is added.
add_library(uavnav_core INTERFACE)
add_library(uavnav_core::uavnav_core ALIAS uavnav_core)
target_include_directories(uavnav_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uavnav_core INTERFACE cxx_std_23)
target_compile_options(uavnav_core INTERFACE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(uavnav_core INTERFACE yaml-cpp::yaml-cpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uavnav_core EXPORT export_uavnav_core)
ament_export_targets(export_uavnav_core HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_result test/test_result.cpp)
  target_link_libraries(test_result uavnav_core::uavnav_core)
endif()

ament_package()
