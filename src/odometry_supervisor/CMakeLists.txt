cmake_minimum_required(VERSION 3.16)
project(odometry_supervisor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/residual_calculator.cpp
  src/supervisor_state_machine.cpp
  src/diagnostic_adapter.cpp
  src/world_alignment.cpp
  src/odometry_alignment_estimator.cpp
  src/alignment_lifecycle_manager.cpp
  src/query_epoch_eligibility.cpp
  src/lio_lifecycle_coordinator.cpp)
add_library(${PROJECT_NAME}::core ALIAS ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_core diagnostic_msgs)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(${PROJECT_NAME}_node src/odometry_supervisor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${navigation_interfaces_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_node
  builtin_interfaces diagnostic_msgs nav_msgs navigation_interfaces rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_residual_calculator.cpp
    test/test_supervisor_state_machine.cpp
    test/test_diagnostic_adapter.cpp
    test/test_world_alignment.cpp
    test/test_odometry_alignment_estimator.cpp
    test/test_alignment_lifecycle_manager.cpp
    test/test_query_epoch_eligibility.cpp
    test/test_lio_lifecycle_coordinator.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}_core)
  ament_target_dependencies(test_${PROJECT_NAME} diagnostic_msgs)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 diagnostic_msgs)
ament_package()
