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
find_package(px4_nav_common REQUIRED)
find_package(px4_ros2_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
    px4_nav_common
    px4_ros2_utils
    rclcpp
    nav_msgs
    sensor_msgs
    px4_msgs
)
# LIO-PX4 bridge core (shared with unit tests) and executable.
add_library(lio_px4_bridge_core STATIC
    src/lio_px4_bridge.cpp
)
target_include_directories(lio_px4_bridge_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(lio_px4_bridge_core
    ${PROJECT_NAME}
)
ament_target_dependencies(lio_px4_bridge_core
    px4_nav_common
    px4_ros2_utils
    rclcpp
    nav_msgs
    diagnostic_msgs
    px4_msgs
)
add_executable(lio_px4_bridge
    src/lio_px4_bridge_main.cpp
)
target_link_libraries(lio_px4_bridge
    lio_px4_bridge_core
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
    px4_nav_common
    px4_ros2_utils
    px4_msgs
    rclcpp
    nav_msgs
    sensor_msgs
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
install(TARGETS global_mapper_core lio_px4_bridge_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(TARGETS ${PROJECT_NAME} global_mapper lio_px4_bridge
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
ament_export_dependencies(px4_nav_common px4_ros2_utils Eigen3 rclcpp nav_msgs sensor_msgs diagnostic_msgs px4_msgs)
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_voxel_pool test/test_voxel_pool.cpp)
    target_link_libraries(test_voxel_pool ${PROJECT_NAME})
    ament_add_gtest(test_voxel_hash_map test/test_voxel_hash_map.cpp)
    target_link_libraries(test_voxel_hash_map ${PROJECT_NAME})
    ament_add_gtest(test_global_mapper test/test_global_mapper.cpp)
    target_link_libraries(test_global_mapper ${PROJECT_NAME} global_mapper_core)
    ament_add_gtest(test_lio_px4_bridge test/test_lio_px4_bridge.cpp)
    target_link_libraries(test_lio_px4_bridge ${PROJECT_NAME} lio_px4_bridge_core)
endif()
ament_package()
