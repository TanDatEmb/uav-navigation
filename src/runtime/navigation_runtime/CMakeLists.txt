cmake_minimum_required(VERSION 3.16)
project(navigation_runtime LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_common REQUIRED)
find_package(navigation_contracts REQUIRED)
find_package(navigation_execution REQUIRED)
find_package(navigation_planning REQUIRED)
find_package(navigation_mapping REQUIRED)
find_package(navigation_world_model REQUIRED)
find_package(rog_map_vendor REQUIRED)
find_package(navigation_planning_backend REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(navigation_runtime_core
  src/mission_dynamics.cpp
  src/mapping_fail_stop.cpp
  src/navigation_runtime_node.cpp
)
target_compile_features(navigation_runtime_core PUBLIC cxx_std_20)
target_compile_options(navigation_runtime_core PRIVATE -Wall -Wextra -O3)
target_include_directories(navigation_runtime_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(navigation_runtime_core PRIVATE
  ${navigation_contracts_INCLUDE_DIRS})
target_link_libraries(navigation_runtime_core PUBLIC
  navigation_planning_backend::navigation_planning_backend
  navigation_world_model::navigation_world_model
  rog_map_vendor::rog_map_vendor
  yaml-cpp::yaml-cpp)
ament_target_dependencies(navigation_runtime_core PUBLIC
  ament_index_cpp
  builtin_interfaces
  diagnostic_msgs
  nav_msgs
  navigation_common
  navigation_contracts
  navigation_execution
  navigation_planning
  navigation_mapping
  rclcpp
  sensor_msgs
)

add_executable(navigation_runtime_node src/navigation_runtime_main.cpp)
target_compile_features(navigation_runtime_node PUBLIC cxx_std_20)
target_compile_options(navigation_runtime_node PRIVATE -Wall -Wextra -O3)
target_link_libraries(navigation_runtime_node PRIVATE navigation_runtime_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS navigation_runtime_core
  EXPORT navigation_runtimeTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS navigation_runtime_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_dependencies(
  ament_index_cpp
  builtin_interfaces
  diagnostic_msgs
  nav_msgs
  navigation_common
  navigation_contracts
  navigation_execution
  navigation_planning
  navigation_mapping
  navigation_world_model
  rclcpp
  rog_map_vendor
  sensor_msgs
  navigation_planning_backend
  yaml-cpp
)
ament_export_targets(navigation_runtimeTargets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_input_pairing test/test_input_pairing.cpp)
  target_include_directories(test_input_pairing PRIVATE include)

  ament_add_gtest(test_world_snapshot_store test/test_world_snapshot_store.cpp)
  target_include_directories(test_world_snapshot_store PRIVATE include)
  target_link_libraries(test_world_snapshot_store
    navigation_world_model::navigation_world_model
    navigation_planning_backend::navigation_planning_backend)
  ament_target_dependencies(test_world_snapshot_store navigation_mapping)
  ament_add_gtest(test_mapping_world_model test/test_mapping_world_model.cpp)
  target_include_directories(test_mapping_world_model PRIVATE include)
  target_compile_definitions(test_mapping_world_model PRIVATE
    NAVIGATION_PLANNER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/planner.yaml")
  target_link_libraries(test_mapping_world_model
    navigation_world_model::navigation_world_model
    rog_map_vendor::rog_map_vendor)
  ament_target_dependencies(test_mapping_world_model navigation_mapping navigation_planning_backend)
  ament_target_dependencies(test_input_pairing nav_msgs navigation_common)
  ament_add_gtest(test_planner_fsm test/test_planner_fsm.cpp)
  target_include_directories(test_planner_fsm PRIVATE include)
  ament_target_dependencies(test_planner_fsm navigation_planning_backend)
  ament_add_gtest(test_planner_runtime_context test/test_planner_runtime_context.cpp)
  ament_target_dependencies(test_planner_runtime_context navigation_planning_backend)
  ament_add_gtest(test_mission_dynamics
    test/test_mission_dynamics.cpp
    src/mission_dynamics.cpp)
  target_include_directories(test_mission_dynamics PRIVATE include)
  target_link_libraries(test_mission_dynamics
    navigation_planning_backend::navigation_planning_backend
    yaml-cpp::yaml-cpp)
  ament_add_gtest(test_navigation_runtime_shutdown
    test/test_navigation_runtime_shutdown.cpp)
  target_compile_definitions(test_navigation_runtime_shutdown PRIVATE
    NAVIGATION_PLANNER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/planner.yaml")
  target_link_libraries(test_navigation_runtime_shutdown navigation_runtime_core)
  ament_add_gtest(test_mapping_fail_stop test/test_mapping_fail_stop.cpp)
  target_link_libraries(test_mapping_fail_stop navigation_runtime_core)
endif()

ament_package()
