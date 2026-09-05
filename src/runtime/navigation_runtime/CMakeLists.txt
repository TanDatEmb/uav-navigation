cmake_minimum_required(VERSION 3.16)
project(navigation_runtime LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep one source-of-truth runtime profile in the workspace. The installed
# package still needs a config file for the default launch argument, so copy
# the canonical workspace profile into the build tree during configuration
# instead of maintaining a second hand-edited YAML file.
set(NAVIGATION_RUNTIME_CANONICAL_CONFIG
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/runtime/mapping.yaml")
if(NOT EXISTS "${NAVIGATION_RUNTIME_CANONICAL_CONFIG}")
  message(FATAL_ERROR
    "Missing canonical runtime profile: ${NAVIGATION_RUNTIME_CANONICAL_CONFIG}")
endif()
configure_file(
  "${NAVIGATION_RUNTIME_CANONICAL_CONFIG}"
  "${CMAKE_CURRENT_BINARY_DIR}/navigation_runtime.yaml"
  COPYONLY)

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
find_package(navigation_mission REQUIRED)
find_package(navigation_world_model REQUIRED)
find_package(navigation_planning_backend REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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
  navigation_world_model::navigation_world_model)
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
  navigation_mission
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
install(EXPORT navigation_runtimeTargets
  FILE navigation_runtimeTargets.cmake
  NAMESPACE navigation_runtime::
  DESTINATION share/${PROJECT_NAME}/cmake)
install(TARGETS navigation_runtime_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/navigation_runtime.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/config/planner.yaml"
  DESTINATION share/${PROJECT_NAME}/config)
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
  navigation_mission
  navigation_world_model
  rclcpp
  sensor_msgs
  navigation_planning_backend
)
ament_export_targets(navigation_runtimeTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_world_snapshot_store test/test_world_snapshot_store.cpp)
  target_include_directories(test_world_snapshot_store PRIVATE include)
  target_link_libraries(test_world_snapshot_store
    navigation_world_model::navigation_world_model
    navigation_planning_backend::navigation_planning_backend)
  ament_target_dependencies(test_world_snapshot_store navigation_mapping)
  ament_add_gtest(test_planner_fsm test/test_planner_fsm.cpp)
  target_include_directories(test_planner_fsm PRIVATE include)
  ament_target_dependencies(test_planner_fsm navigation_planning navigation_contracts builtin_interfaces)
  ament_add_gtest(test_planning_worker test/test_planning_worker.cpp)
  target_include_directories(test_planning_worker PRIVATE include)
  ament_target_dependencies(test_planning_worker navigation_planning builtin_interfaces)
  ament_add_gtest(test_mission_dynamics
    test/test_mission_dynamics.cpp
    src/mission_dynamics.cpp)
  target_include_directories(test_mission_dynamics PRIVATE include)
  target_link_libraries(test_mission_dynamics
    navigation_planning_backend::navigation_planning_backend
    navigation_mission::navigation_mission)
  ament_target_dependencies(test_mission_dynamics navigation_mission)
  ament_add_gtest(test_navigation_runtime_shutdown
    test/test_navigation_runtime_shutdown.cpp)
  target_compile_definitions(test_navigation_runtime_shutdown PRIVATE
    NAVIGATION_PLANNER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/planner.yaml"
    NAVIGATION_HANDOVER_MISSION_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/runtime/missions/open.yaml")
  target_link_libraries(test_navigation_runtime_shutdown navigation_runtime_core)
  ament_add_gtest(test_mapping_fail_stop test/test_mapping_fail_stop.cpp)
  target_link_libraries(test_mapping_fail_stop navigation_runtime_core)
  ament_add_gtest(test_kinematic_derivative_estimator
    test/test_kinematic_derivative_estimator.cpp)
  target_include_directories(test_kinematic_derivative_estimator PRIVATE include)
  target_link_libraries(test_kinematic_derivative_estimator
    navigation_planning::navigation_planning)
  ament_add_gtest(test_execution_episode
    test/test_execution_episode.cpp)
  target_include_directories(test_execution_episode PRIVATE include)
  ament_target_dependencies(test_execution_episode navigation_planning builtin_interfaces)
  ament_add_gtest(test_certified_continuation
    test/test_certified_continuation.cpp)
  target_include_directories(test_certified_continuation PRIVATE include)
  ament_target_dependencies(test_certified_continuation navigation_planning)
endif()

ament_package()
