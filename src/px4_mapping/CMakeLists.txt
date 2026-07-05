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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
    px4_common
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(px4_common Eigen3 rclcpp geometry_msgs nav_msgs sensor_msgs)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_voxel_pool test/test_voxel_pool.cpp)
    target_link_libraries(test_voxel_pool ${PROJECT_NAME})
    ament_add_gtest(test_voxel_hash_map test/test_voxel_hash_map.cpp)
    target_link_libraries(test_voxel_hash_map ${PROJECT_NAME})
endif()

ament_package()
