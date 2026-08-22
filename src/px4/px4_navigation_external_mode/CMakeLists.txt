cmake_minimum_required(VERSION 3.16)
project(px4_navigation_external_mode LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(mars_quadrotor_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_contract
  src/super_polynomial.cpp
  src/velocity_tracker.cpp
  src/mission.cpp
  src/mission_controller.cpp)
target_include_directories(${PROJECT_NAME}_contract PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_contract PUBLIC Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME}_contract PUBLIC yaml-cpp::yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_contract PUBLIC diagnostic_msgs geometry_msgs mars_quadrotor_msgs navigation_interfaces nav_msgs)
target_compile_options(${PROJECT_NAME}_contract PRIVATE -Wall -Wextra -Wpedantic)

add_executable(${PROJECT_NAME}_node
  src/navigation_mode_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_contract
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp)
ament_target_dependencies(${PROJECT_NAME}_node diagnostic_msgs navigation_interfaces nav_msgs rclcpp std_msgs)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_contract
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 diagnostic_msgs geometry_msgs mars_quadrotor_msgs navigation_interfaces nav_msgs px4_ros2_cpp rclcpp std_msgs yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission test/test_mission.cpp)
  target_link_libraries(test_mission ${PROJECT_NAME}_contract)
  ament_target_dependencies(test_mission geometry_msgs navigation_interfaces nav_msgs)
  ament_add_gtest(test_super_polynomial test/test_super_polynomial.cpp)
  target_link_libraries(test_super_polynomial ${PROJECT_NAME}_contract)
  ament_target_dependencies(test_super_polynomial mars_quadrotor_msgs)
endif()

ament_package()
