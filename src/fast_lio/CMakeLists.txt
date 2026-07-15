cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Livox driver (optional - can build without for testing)
find_package(livox_ros_driver2 QUIET)
if(NOT livox_ros_driver2_FOUND)
  message(WARNING "livox_ros_driver2 not found, using mock messages")
  set(LIVOX_FOUND FALSE)
else()
  set(LIVOX_FOUND TRUE)
endif()

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)

# Sophus (optional) - check only, apply after target creation
find_package(Sophus QUIET)
if(NOT Sophus_FOUND)
  message(WARNING "Sophus not found, using Eigen fallback for SE3")
  set(USE_EIGEN_SE3 TRUE)
else()
  message(STATUS "Found Sophus")
endif()

# PCL
find_package(PCL REQUIRED COMPONENTS common io filters)

# OpenMP for parallel processing (optional)
find_package(OpenMP)
if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_definitions(-DMP_EN)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Core library
add_library(${PROJECT_NAME}_core SHARED
  src/map_builder/ieskf.cpp
  src/map_builder/imu_processor.cpp
  src/map_builder/lidar_processor.cpp
  src/map_builder/map_builder.cpp
  src/map_builder/pcl_tree_backend.cpp
  src/map_builder/ikd_tree.cpp
  src/map_builder/ikd_tree_backend.cpp
  src/utils.cpp
)

# Add pthread for ikd-Tree
target_link_libraries(${PROJECT_NAME}_core
  pthread
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Apply Sophus/Eigen fallback AFTER target creation
if(USE_EIGEN_SE3)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC USE_EIGEN_SE3)
else()
  target_include_directories(${PROJECT_NAME}_core PUBLIC ${Sophus_INCLUDE_DIRS})
endif()

set(FAST_LIO_DEPS
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
)

if(LIVOX_FOUND)
  list(APPEND FAST_LIO_DEPS livox_ros_driver2)
  target_compile_definitions(${PROJECT_NAME}_core PRIVATE LIVOX_ROS2_FOUND)
endif()

ament_target_dependencies(${PROJECT_NAME}_core
  ${FAST_LIO_DEPS}
)
target_link_libraries(${PROJECT_NAME}_core
  ${PCL_LIBRARIES}
)
if(OpenMP_FOUND)
  target_link_libraries(${PROJECT_NAME}_core OpenMP::OpenMP_CXX)
endif()

# Executable
add_executable(${PROJECT_NAME}_node src/fast_lio_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

# Apply same compile definitions to executable
if(USE_EIGEN_SE3)
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_EIGEN_SE3)
endif()
if(LIVOX_FOUND)
  target_compile_definitions(${PROJECT_NAME}_node PRIVATE LIVOX_ROS2_FOUND)
endif()

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME}_core HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ieskf test/test_ieskf.cpp)
  target_link_libraries(test_ieskf ${PROJECT_NAME}_core)
endif()

ament_package()
