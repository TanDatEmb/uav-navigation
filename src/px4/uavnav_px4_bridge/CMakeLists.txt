cmake_minimum_required(VERSION 3.22)
project(uavnav_px4_bridge LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(uavnav_core REQUIRED)
find_package(Eigen3 REQUIRED)

# No ROS (rclcpp) and no px4_msgs dependency: this library must stay usable outside any node.
# EvSample maps 1:1 onto px4_msgs::msg::VehicleOdometry, the node copies it field by field.
set(UAVNAV_PX4_BRIDGE_SOURCES
  src/alignment.cpp
  src/config.cpp
  src/ev_encoder.cpp
  src/px4_time.cpp)

add_library(uavnav_px4_bridge_core STATIC ${UAVNAV_PX4_BRIDGE_SOURCES})
add_library(uavnav_px4_bridge::uavnav_px4_bridge_core ALIAS uavnav_px4_bridge_core)
target_include_directories(uavnav_px4_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uavnav_px4_bridge_core PUBLIC cxx_std_23)
# Strict warnings apply to this package only; they must not leak to consumers.
set(UAVNAV_PX4_BRIDGE_STRICT_WARNINGS -Wall -Wextra -Wpedantic -Werror)
target_compile_options(uavnav_px4_bridge_core PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})
set_target_properties(uavnav_px4_bridge_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(uavnav_px4_bridge_core PUBLIC uavnav_core::uavnav_core Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uavnav_px4_bridge_core EXPORT export_uavnav_px4_bridge_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_uavnav_px4_bridge_core HAS_LIBRARY_TARGET)
ament_export_dependencies(uavnav_core Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_frames test_px4_time test_ev_encoder test_alignment test_alignment_config)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 300)
    target_link_libraries(${test_name} uavnav_px4_bridge::uavnav_px4_bridge_core)
    target_compile_options(${test_name} PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})
  endforeach()
endif()

ament_package()
