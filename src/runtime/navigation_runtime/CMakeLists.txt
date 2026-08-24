cmake_minimum_required(VERSION 3.16)
project(navigation_runtime LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(super_planner_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(super_navigation_node
  src/mission_dynamics.cpp
  src/super_navigation_main.cpp
  src/super_navigation_node.cpp
)
target_compile_features(super_navigation_node PUBLIC cxx_std_20)
target_compile_options(super_navigation_node PRIVATE -Wall -Wextra -O3)
target_include_directories(super_navigation_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(super_navigation_node PRIVATE
  super_planner_vendor::super_planner_vendor
  yaml-cpp::yaml-cpp)
ament_target_dependencies(super_navigation_node PUBLIC
  ament_index_cpp
  diagnostic_msgs
  mars_quadrotor_msgs
  nav_msgs
  navigation_interfaces
  rclcpp
  sensor_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS super_navigation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_dependencies(navigation_interfaces mars_quadrotor_msgs super_planner_vendor)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_input_pairing test/test_input_pairing.cpp)
  target_include_directories(test_input_pairing PRIVATE include)
  ament_target_dependencies(test_input_pairing nav_msgs)
  ament_add_gtest(test_planner_fsm test/test_planner_fsm.cpp)
  target_include_directories(test_planner_fsm PRIVATE include)
  ament_target_dependencies(test_planner_fsm super_planner_vendor)
  ament_add_gtest(test_command_clock test/test_command_clock.cpp)
  ament_target_dependencies(test_command_clock super_planner_vendor)
  ament_add_gtest(test_mission_dynamics
    test/test_mission_dynamics.cpp
    src/mission_dynamics.cpp)
  target_include_directories(test_mission_dynamics PRIVATE include)
  target_link_libraries(test_mission_dynamics
    super_planner_vendor::super_planner_vendor
    yaml-cpp::yaml-cpp)
endif()

ament_package()
