cmake_minimum_required(VERSION 3.16)
project(px4_odometry_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(navigation_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(navigation_contracts REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../external/px4_msgs/package.xml")
  message(FATAL_ERROR
    "px4_msgs package not found; initialize submodule with: "
    "git submodule update --init --recursive")
endif()

add_library(${PROJECT_NAME}_core
  src/frame_converter.cpp
  src/reference_point_converter.cpp
  src/external_odometry_conversion.cpp
  src/external_odometry_gate.cpp
  src/geometric_jump_continuity.cpp
  src/geometric_jump_latch.cpp
  src/timestamp_conversion.cpp
  src/reset_compensator.cpp
  src/odometry_ring_buffer.cpp
  src/time_validator.cpp)
add_library(${PROJECT_NAME}::core ALIAS ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_core navigation_common nav_msgs)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(${PROJECT_NAME}_node src/px4_odometry_bridge_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${navigation_contracts_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_node
  diagnostic_msgs nav_msgs rclcpp rosidl_default_runtime navigation_contracts px4_msgs)

add_executable(${PROJECT_NAME}_external_node src/px4_external_odometry_bridge_node.cpp)
target_link_libraries(${PROJECT_NAME}_external_node ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_external_node PRIVATE ${navigation_contracts_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_external_node
  diagnostic_msgs nav_msgs rclcpp rosidl_default_runtime navigation_contracts px4_msgs)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_external_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/test_px4_odometry_bridge.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_core)
  ament_target_dependencies(test_${PROJECT_NAME}
    diagnostic_msgs nav_msgs rclcpp rosidl_default_runtime navigation_contracts px4_msgs)
  target_include_directories(test_${PROJECT_NAME} PRIVATE test)
  ament_add_gtest(test_external_odometry_conversion
    test/test_external_odometry_conversion.cpp)
  target_link_libraries(test_external_odometry_conversion ${PROJECT_NAME}_core)
  ament_target_dependencies(test_external_odometry_conversion
    nav_msgs rclcpp rosidl_default_runtime px4_msgs)
  ament_add_gtest(test_external_odometry_gate
    test/test_external_odometry_gate.cpp)
  target_link_libraries(test_external_odometry_gate ${PROJECT_NAME}_core)
  ament_target_dependencies(test_external_odometry_gate
    rclcpp rosidl_default_runtime)
  ament_add_gtest(test_geometric_jump_latch
    test/test_geometric_jump_latch.cpp)
  target_link_libraries(test_geometric_jump_latch ${PROJECT_NAME}_core)
  ament_target_dependencies(test_geometric_jump_latch
    rclcpp rosidl_default_runtime)
  ament_add_gtest(test_geometric_jump_continuity
    test/test_geometric_jump_continuity.cpp)
  target_link_libraries(test_geometric_jump_continuity ${PROJECT_NAME}_core)
  ament_target_dependencies(test_geometric_jump_continuity
    rclcpp rosidl_default_runtime)
  ament_add_gtest(test_timestamp_conversion
    test/test_timestamp_conversion.cpp)
  target_link_libraries(test_timestamp_conversion ${PROJECT_NAME}_core)
  ament_target_dependencies(test_timestamp_conversion
    rclcpp rosidl_default_runtime)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(navigation_common Eigen3 diagnostic_msgs nav_msgs rclcpp navigation_contracts px4_msgs)
ament_package()
