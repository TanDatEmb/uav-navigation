cmake_minimum_required(VERSION 3.22)
project(uavnav_px4_bridge LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(uavnav_core REQUIRED)
find_package(Eigen3 REQUIRED)

# No ROS (rclcpp) and no px4_msgs dependency: this library must stay usable outside any node.
# EvSample maps 1:1 onto px4_msgs::msg::VehicleOdometry, the node copies it field by field (ros_conversion.hpp).
set(UAVNAV_PX4_BRIDGE_SOURCES
  src/alignment.cpp
  src/bridge.cpp
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

# --- uavnav_px4_bridge_ros + px4_bridge_node: the ROS node (S1b Task 10; SYSTEM_DESIGN §4.1, §6.4, §6.5, D33/O19-O22) ---
# Message conversion at the boundary (a static library, linked by the executable and the tests; not exported) and the
# `px4_bridge_node` executable in lib/uavnav_px4_bridge. The core target above links no rclcpp and no message package;
# test_core_target_has_no_ros pins that. ROS may appear only in the files listed under ros_allowed of this package in
# tools/uavnav/arch_rules.yaml, which is why the node class lives in src/px4_bridge_node.cpp and the node test drives
# the executable as a process.
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uavnav_interfaces REQUIRED)
find_package(uavnav_ros REQUIRED)
add_library(uavnav_px4_bridge_ros STATIC src/ros_conversion.cpp)
target_compile_options(uavnav_px4_bridge_ros PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})
set_target_properties(uavnav_px4_bridge_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
# The C++ typesupport target of a message package carries its headers and the library publishers need.
target_link_libraries(uavnav_px4_bridge_ros PUBLIC
  uavnav_px4_bridge_core
  uavnav_ros::uavnav_ros
  rclcpp::rclcpp
  px4_msgs::px4_msgs__rosidl_typesupport_cpp
  uavnav_interfaces::uavnav_interfaces__rosidl_typesupport_cpp)

add_executable(px4_bridge_node src/px4_bridge_node.cpp)
target_compile_options(px4_bridge_node PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})
target_link_libraries(px4_bridge_node PRIVATE uavnav_px4_bridge_ros)
install(TARGETS px4_bridge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_frames test_px4_time test_ev_encoder test_alignment test_alignment_config test_bridge)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 300)
    target_link_libraries(${test_name} uavnav_px4_bridge::uavnav_px4_bridge_core)
    target_compile_options(${test_name} PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})
    # test_alignment_config loads config/px4_bridge/sim.yaml (D22); the path is relative to this package.
    target_compile_definitions(${test_name} PRIVATE
      UAVNAV_REPO_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../../config")
  endforeach()

  # Pure message conversion: no rclcpp context, gtest's own main.
  ament_add_gtest(test_ros_conversion test/test_ros_conversion.cpp TIMEOUT 120)
  target_link_libraries(test_ros_conversion uavnav_px4_bridge_ros)
  target_compile_options(test_ros_conversion PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})

  # The core target stays ROS-free: its sources and headers, and the libraries it links (read at configure time).
  get_target_property(UAVNAV_PX4_BRIDGE_CORE_LIBS uavnav_px4_bridge_core LINK_LIBRARIES)
  string(REPLACE ";" "," UAVNAV_PX4_BRIDGE_CORE_LIBS "${UAVNAV_PX4_BRIDGE_CORE_LIBS}")
  ament_add_gtest(test_core_target_has_no_ros test/test_core_target_has_no_ros.cpp TIMEOUT 60)
  target_link_libraries(test_core_target_has_no_ros uavnav_px4_bridge::uavnav_px4_bridge_core)
  target_compile_options(test_core_target_has_no_ros PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})
  target_compile_definitions(test_core_target_has_no_ros PRIVATE
    UAVNAV_PX4_BRIDGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    UAVNAV_PX4_BRIDGE_CORE_LIBS="${UAVNAV_PX4_BRIDGE_CORE_LIBS}")

  # The node as a process: the test is a ROS peer in its own DDS domain and runs the executable (own main, which
  # sets that domain in the environment the child inherits). RESOURCE_LOCK ros_dds as the other in-process ROS
  # tests (the name uavnav_ros and uavnav_lio use), so ctest never runs two of them at once.
  ament_add_gtest(test_px4_bridge_node test/test_px4_bridge_node.cpp TIMEOUT 300 SKIP_LINKING_MAIN_LIBRARIES)
  add_dependencies(test_px4_bridge_node px4_bridge_node)
  target_link_libraries(test_px4_bridge_node uavnav_px4_bridge_ros gtest)
  target_compile_options(test_px4_bridge_node PRIVATE ${UAVNAV_PX4_BRIDGE_STRICT_WARNINGS})
  target_compile_definitions(test_px4_bridge_node PRIVATE
    UAVNAV_PX4_BRIDGE_NODE_EXE="$<TARGET_FILE:px4_bridge_node>"
    UAVNAV_PX4_BRIDGE_SIM_YAML="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/px4_bridge/sim.yaml")
  set_tests_properties(test_px4_bridge_node PROPERTIES RESOURCE_LOCK ros_dds)
endif()

ament_package()
