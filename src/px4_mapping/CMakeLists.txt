cmake_minimum_required(VERSION 3.8)
project(px4_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_common REQUIRED)
find_package(px4_ros_com REQUIRED)
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
    px4_common
    px4_ros_com
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    px4_msgs
)

# Add executable for ned_transform_node
add_executable(ned_transform_node
    src/ned_transform_node.cpp
    src/ned_transform_node_main.cpp
)
target_include_directories(ned_transform_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ned_transform_node
    ${PROJECT_NAME}
)
ament_target_dependencies(ned_transform_node
    px4_common
    px4_ros_com
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    px4_msgs
)

# FAST-LIO2 adapter node
add_executable(fast_lio2_node
    src/fast_lio2_node.cpp
    src/fast_lio2_node_main.cpp
)
target_include_directories(fast_lio2_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(fast_lio2_node
    ${PROJECT_NAME}
)
ament_target_dependencies(fast_lio2_node
    px4_common
    rclcpp
    nav_msgs
    sensor_msgs
    px4_msgs
)

# Voxel map manager core library (shared with tests and the node executable)
add_library(voxmap_manager_core STATIC
    src/voxmap_manager_node.cpp
)
target_include_directories(voxmap_manager_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(voxmap_manager_core
    Eigen3::Eigen
)
ament_target_dependencies(voxmap_manager_core
    px4_common
    px4_msgs
    px4_ros_com
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2_ros
)

# Voxel map manager node executable
add_executable(voxmap_manager_node
    src/voxmap_manager_node_main.cpp
)
target_include_directories(voxmap_manager_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(voxmap_manager_node
    voxmap_manager_core
)

install(TARGETS voxmap_manager_core
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME} fast_lio2_node ned_transform_node voxmap_manager_node
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
ament_export_dependencies(px4_common px4_ros_com Eigen3 rclcpp geometry_msgs nav_msgs sensor_msgs px4_msgs tf2_ros)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_voxel_pool test/test_voxel_pool.cpp)
    target_link_libraries(test_voxel_pool ${PROJECT_NAME})
    ament_add_gtest(test_voxel_hash_map test/test_voxel_hash_map.cpp)
    target_link_libraries(test_voxel_hash_map ${PROJECT_NAME})
    ament_add_gtest(test_voxmap_manager_node test/test_voxmap_manager_node.cpp)
    target_link_libraries(test_voxmap_manager_node ${PROJECT_NAME} voxmap_manager_core)
endif()

ament_package()
