cmake_minimum_required(VERSION 3.16)
project(px4_navigation_external_mode LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(navigation_common REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(navigation_contracts REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(PX4_SETPOINT_CORE_HEADERS
  include/px4_navigation_external_mode/command_admission_assessment.hpp
  include/px4_navigation_external_mode/command_acceptance_gate.hpp
  include/px4_navigation_external_mode/tracking_envelope.hpp
  include/px4_navigation_external_mode/velocity_only_continuity.hpp
  include/px4_navigation_external_mode/px4_tracking_adapter.hpp
  include/px4_navigation_external_mode/certified_command_handoff.hpp
  include/px4_navigation_external_mode/mission_command_identity.hpp
  include/px4_navigation_external_mode/planner_recovery.hpp
  include/px4_navigation_external_mode/local_frame_alignment.hpp
  include/px4_navigation_external_mode/estimator_bias_observation.hpp
  include/px4_navigation_external_mode/frame_transform_contract.hpp
  include/px4_navigation_external_mode/navigation_input_validation.hpp
  include/px4_navigation_external_mode/runtime_metrics_policy.hpp)

# Compile every pure header through one ROS/PX4-free translation unit.  The
# target is intentionally private to this package: adapter and pure tests use
# it, but no installed/exported target is needed by downstream packages.
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/px4_setpoint_core_anchor.cpp"
  CONTENT [=[
#include "px4_navigation_external_mode/command_admission_assessment.hpp"
#include "px4_navigation_external_mode/command_acceptance_gate.hpp"
#include "px4_navigation_external_mode/tracking_envelope.hpp"
#include "px4_navigation_external_mode/velocity_only_continuity.hpp"
#include "px4_navigation_external_mode/px4_tracking_adapter.hpp"
#include "px4_navigation_external_mode/certified_command_handoff.hpp"
#include "px4_navigation_external_mode/mission_command_identity.hpp"
#include "px4_navigation_external_mode/planner_recovery.hpp"
#include "px4_navigation_external_mode/local_frame_alignment.hpp"
#include "px4_navigation_external_mode/estimator_bias_observation.hpp"
#include "px4_navigation_external_mode/frame_transform_contract.hpp"
#include "px4_navigation_external_mode/navigation_input_validation.hpp"
#include "px4_navigation_external_mode/runtime_metrics_policy.hpp"
namespace px4_navigation_external_mode { void px4_setpoint_core_anchor() {} }
]=])
add_library(px4_setpoint_core STATIC
  "${CMAKE_CURRENT_BINARY_DIR}/px4_setpoint_core_anchor.cpp"
  ${PX4_SETPOINT_CORE_HEADERS})
target_include_directories(px4_setpoint_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${navigation_contracts_INCLUDE_DIRS})
target_link_libraries(px4_setpoint_core PUBLIC Eigen3::Eigen)
ament_target_dependencies(px4_setpoint_core PUBLIC navigation_common navigation_contracts)
target_compile_options(px4_setpoint_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(${PROJECT_NAME}_adapter STATIC
  src/navigation_mode_node.cpp)
target_include_directories(${PROJECT_NAME}_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_adapter PUBLIC
  ${navigation_contracts_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_adapter
  px4_setpoint_core
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp)
ament_target_dependencies(${PROJECT_NAME}_adapter navigation_common diagnostic_msgs navigation_contracts nav_msgs px4_msgs px4_ros2_cpp rclcpp std_msgs)
target_compile_options(${PROJECT_NAME}_adapter PRIVATE -Wall -Wextra -Wpedantic)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_adapter)
ament_target_dependencies(${PROJECT_NAME}_node px4_ros2_cpp rclcpp)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME test_mission_authority_static
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/check_mission_authority_cut.py)
  add_test(NAME test_setpoint_core_layout
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_setpoint_core_layout.py
      ${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_mode_progression test/test_navigation_mode_progression.cpp)
  target_link_libraries(test_navigation_mode_progression ${PROJECT_NAME}_adapter)
  ament_target_dependencies(test_navigation_mode_progression px4_ros2_cpp rclcpp)
  ament_add_gtest(test_navigation_command test/test_navigation_command.cpp)
  target_include_directories(test_navigation_command PRIVATE
    include ${navigation_contracts_INCLUDE_DIRS})
  target_link_libraries(test_navigation_command
    px4_setpoint_core Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp)
  ament_target_dependencies(test_navigation_command navigation_common navigation_contracts nav_msgs)
  ament_add_gtest(test_tracking_envelope test/test_tracking_envelope.cpp)
  target_include_directories(test_tracking_envelope PRIVATE include)
  target_link_libraries(test_tracking_envelope px4_setpoint_core)
  ament_add_gtest(test_local_frame_alignment test/test_local_frame_alignment.cpp)
  target_include_directories(test_local_frame_alignment PRIVATE include)
  target_link_libraries(test_local_frame_alignment px4_setpoint_core)
  ament_target_dependencies(test_local_frame_alignment navigation_common)
  ament_add_gtest(test_estimator_bias_observation test/test_estimator_bias_observation.cpp)
  target_include_directories(test_estimator_bias_observation PRIVATE include)
  target_link_libraries(test_estimator_bias_observation px4_setpoint_core Eigen3::Eigen)
  ament_add_gtest(test_frame_transform_contract test/test_frame_transform_contract.cpp)
  target_include_directories(test_frame_transform_contract PRIVATE include)
  target_link_libraries(test_frame_transform_contract px4_setpoint_core Eigen3::Eigen)
  ament_add_gtest(test_px4_tracking_adapter test/test_px4_tracking_adapter.cpp)
  target_include_directories(test_px4_tracking_adapter PRIVATE include)
  target_link_libraries(test_px4_tracking_adapter px4_setpoint_core)
  ament_add_gtest(test_velocity_only_continuity test/test_velocity_only_continuity.cpp)
  target_include_directories(test_velocity_only_continuity PRIVATE include)
  target_link_libraries(test_velocity_only_continuity px4_setpoint_core)
  ament_add_gtest(test_paired_node_lifetime test/test_paired_node_lifetime.cpp)
  target_include_directories(test_paired_node_lifetime PRIVATE include)
endif()

ament_package()
