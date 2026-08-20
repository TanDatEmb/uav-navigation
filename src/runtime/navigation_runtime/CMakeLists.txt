cmake_minimum_required(VERSION 3.16)
project(navigation_runtime LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(navigation_mapping REQUIRED)
find_package(navigation_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(navigation_runtime_node
  src/main.cpp
  src/navigation_runtime_node.cpp
)
target_compile_features(navigation_runtime_node PUBLIC cxx_std_20)
target_include_directories(navigation_runtime_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(navigation_runtime_node PUBLIC
  navigation_mapping::navigation_mapping_core
  navigation_planning::navigation_planning
)
ament_target_dependencies(navigation_runtime_node PUBLIC
  builtin_interfaces
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  navigation_interfaces
  navigation_mapping
  navigation_planning
  rclcpp
  sensor_msgs
  visualization_msgs
)
target_compile_options(navigation_runtime_node PRIVATE -Wall -Wextra)

ament_add_gtest(test_local_goal_selector test/test_local_goal_selector.cpp)
target_include_directories(test_local_goal_selector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(test_local_goal_selector navigation_mapping)
target_compile_features(test_local_goal_selector PUBLIC cxx_std_20)
target_compile_options(test_local_goal_selector PRIVATE -Wall -Wextra)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS navigation_runtime_node RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(navigation_mapping navigation_planning navigation_interfaces visualization_msgs)
ament_package()
