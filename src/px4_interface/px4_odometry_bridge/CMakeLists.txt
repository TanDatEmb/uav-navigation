cmake_minimum_required(VERSION 3.16)
project(px4_odometry_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(px4_msgs QUIET)

file(GLOB BRIDGE_CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
list(FILTER BRIDGE_CORE_SOURCES EXCLUDE REGEX ".*px4_odometry_bridge_node\\.cpp$")
add_library(${PROJECT_NAME}_core ${BRIDGE_CORE_SOURCES})
add_library(${PROJECT_NAME}::core ALIAS ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

if(px4_msgs_FOUND)
  add_executable(${PROJECT_NAME}_node src/px4_odometry_bridge_node.cpp)
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
  ament_target_dependencies(${PROJECT_NAME}_node
    diagnostic_msgs nav_msgs rclcpp rosidl_default_runtime navigation_interfaces px4_msgs)
  install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "px4_msgs not found: building ROS-independent P0.7 bridge core only")
endif()

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/test_px4_odometry_bridge.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_core)
  target_include_directories(test_${PROJECT_NAME} PRIVATE test)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 diagnostic_msgs nav_msgs rclcpp navigation_interfaces)
ament_package()
