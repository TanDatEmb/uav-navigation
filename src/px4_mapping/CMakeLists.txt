cmake_minimum_required(VERSION 3.8)
project(px4_mapping)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_navigation_common REQUIRED)
find_package(px4_ros2_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
    px4_navigation_common
    px4_ros2_utils
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    px4_msgs
)
# Localization bridge executable
add_executable(localization_bridge
    src/localization_bridge.cpp
    src/localization_bridge_main.cpp
)
target_include_directories(localization_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(localization_bridge
    ${PROJECT_NAME}
)
ament_target_dependencies(localization_bridge
    px4_navigation_common
    px4_ros2_utils
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    px4_msgs
)
# LiDAR odometry executable
add_executable(lidar_odometry
    src/lidar_odometry.cpp
    src/lidar_odometry_main.cpp
)
target_include_directories(lidar_odometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_odometry
    ${PROJECT_NAME}
)
ament_target_dependencies(lidar_odometry
    px4_navigation_common
    px4_ros2_utils
    rclcpp
    nav_msgs
    sensor_msgs
    px4_msgs
)
# Global mapper core library (shared with tests and the node executable)
add_library(global_mapper_core STATIC
    src/global_mapper.cpp
)
target_include_directories(global_mapper_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(global_mapper_core
    Eigen3::Eigen
)
ament_target_dependencies(global_mapper_core
    px4_navigation_common
    px4_ros2_utils
    px4_msgs
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_ros
)
# Global mapper node executable
add_executable(global_mapper
    src/global_mapper_main.cpp
)
target_include_directories(global_mapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(global_mapper
    global_mapper_core
)
install(TARGETS global_mapper_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(TARGETS ${PROJECT_NAME} lidar_odometry localization_bridge global_mapper
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
    DESTINATION include
)
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(px4_navigation_common px4_ros2_utils Eigen3 rclcpp geometry_msgs nav_msgs sensor_msgs px4_msgs tf2_ros)
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_voxel_pool test/test_voxel_pool.cpp)
    target_link_libraries(test_voxel_pool ${PROJECT_NAME})
    ament_add_gtest(test_voxel_hash_map test/test_voxel_hash_map.cpp)
    target_link_libraries(test_voxel_hash_map ${PROJECT_NAME})
    ament_add_gtest(test_global_mapper test/test_global_mapper.cpp)
    target_link_libraries(test_global_mapper ${PROJECT_NAME} global_mapper_core)
endif()
ament_package()
