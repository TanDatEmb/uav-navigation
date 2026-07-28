cmake_minimum_required(VERSION 3.16)
project(fast_lio_ros LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(fast_lio_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}
  src/parameter_loader.cpp
  src/qos_profiles.cpp
  src/ros_imu_adapter.cpp
  src/ros_lidar_adapter.cpp
  src/ros_livox_custom_adapter.cpp
  src/ros_output_publisher.cpp
  src/ros_time_converter.cpp
  src/ros_transform_publisher.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs fast_lio_core geometry_msgs livox_ros_driver2 nav_msgs
  rclcpp sensor_msgs tf2_ros)

add_executable(fast_lio_node src/fast_lio_node.cpp src/main.cpp)
target_link_libraries(fast_lio_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} fast_lio_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name IN ITEMS
      ros_time_converter ros_imu_adapter ros_lidar_adapter
      ros_livox_custom_adapter parameter_loader qos_profiles)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fast_lio_core livox_ros_driver2)
ament_package()
