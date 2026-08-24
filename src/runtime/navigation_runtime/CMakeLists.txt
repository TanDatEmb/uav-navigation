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
find_package(navigation_world_model REQUIRED)
find_package(rog_map_vendor REQUIRED)
find_package(super_planner_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(navigation_runtime_node
  src/mission_dynamics.cpp
  src/mapping_fail_stop.cpp
  src/super_navigation_node.cpp
)
target_compile_features(navigation_runtime_node PUBLIC cxx_std_20)
target_compile_options(navigation_runtime_node PRIVATE -Wall -Wextra -O3)
target_include_directories(navigation_runtime_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(navigation_runtime_node PUBLIC
  super_planner_vendor::super_planner_vendor
  navigation_world_model::navigation_world_model
  rog_map_vendor::rog_map_vendor
  yaml-cpp::yaml-cpp)
ament_target_dependencies(navigation_runtime_node PUBLIC
  ament_index_cpp
  builtin_interfaces
  diagnostic_msgs
  mars_quadrotor_msgs
  nav_msgs
  navigation_interfaces
  rclcpp
  sensor_msgs
)

add_executable(super_navigation_node src/super_navigation_main.cpp)
target_compile_features(super_navigation_node PUBLIC cxx_std_20)
target_compile_options(super_navigation_node PRIVATE -Wall -Wextra -O3)
target_link_libraries(super_navigation_node PRIVATE navigation_runtime_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS navigation_runtime_node
  EXPORT navigation_runtimeTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS super_navigation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_dependencies(
  ament_index_cpp
  builtin_interfaces
  diagnostic_msgs
  mars_quadrotor_msgs
  nav_msgs
  navigation_interfaces
  navigation_world_model
  rclcpp
  rog_map_vendor
  sensor_msgs
  super_planner_vendor
  yaml-cpp
)
ament_export_targets(navigation_runtimeTargets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_input_pairing test/test_input_pairing.cpp)
  target_include_directories(test_input_pairing PRIVATE include)

  ament_add_gtest(test_observation_accounting test/test_observation_accounting.cpp)
  target_include_directories(test_observation_accounting PRIVATE include)

  ament_add_gtest(test_timestamp_freshness test/test_timestamp_freshness.cpp)
  target_include_directories(test_timestamp_freshness PRIVATE include)
  ament_add_gtest(test_world_snapshot_store test/test_world_snapshot_store.cpp)
  target_include_directories(test_world_snapshot_store PRIVATE include)
  target_link_libraries(test_world_snapshot_store
    navigation_world_model::navigation_world_model
    super_planner_vendor::super_planner_vendor)
  ament_add_gtest(test_rog_world_model_adapter test/test_rog_world_model_adapter.cpp)
  target_include_directories(test_rog_world_model_adapter PRIVATE include)
  target_compile_definitions(test_rog_world_model_adapter PRIVATE
    SUPER_PRODUCT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/super_planner.yaml")
  target_link_libraries(test_rog_world_model_adapter
    navigation_world_model::navigation_world_model
    rog_map_vendor::rog_map_vendor)
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
