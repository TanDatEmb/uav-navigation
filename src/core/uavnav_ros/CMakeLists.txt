cmake_minimum_required(VERSION 3.22)
project(uavnav_ros LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(uavnav_core REQUIRED)
find_package(uavnav_interfaces REQUIRED)

# The shared ROS shell helpers (D33/O19): time conversion, RosEventSink, product QoS. This is the one
# package under src/core that depends on rclcpp; uavnav_core stays ROS-free.
add_library(uavnav_ros STATIC src/ros_event_sink.cpp)
add_library(uavnav_ros::uavnav_ros ALIAS uavnav_ros)
target_include_directories(uavnav_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uavnav_ros PUBLIC cxx_std_23)
# Strict warnings apply to this package only; they must not leak to consumers.
set(UAVNAV_ROS_STRICT_WARNINGS -Wall -Wextra -Wpedantic -Werror)
target_compile_options(uavnav_ros PRIVATE ${UAVNAV_ROS_STRICT_WARNINGS})
# Static code is linked into shared libraries and nodes of other packages.
set_target_properties(uavnav_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(uavnav_ros PUBLIC uavnav_core::uavnav_core)
ament_target_dependencies(uavnav_ros PUBLIC rclcpp builtin_interfaces uavnav_interfaces)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uavnav_ros EXPORT export_uavnav_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_uavnav_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp builtin_interfaces uavnav_core uavnav_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The rclcpp tests share one main (test/ros_test_main.cpp) that initialises rclcpp once on a
  # per-process DDS domain restricted to localhost, so parallel ctest runs do not see each other.
  foreach(test_name test_time_msg test_ros_event_sink test_qos)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 300 SKIP_LINKING_MAIN_LIBRARIES)
    target_sources(${test_name} PRIVATE test/ros_test_main.cpp)
    target_link_libraries(${test_name} uavnav_ros::uavnav_ros)
    target_compile_options(${test_name} PRIVATE ${UAVNAV_ROS_STRICT_WARNINGS})
  endforeach()
endif()

ament_package()
