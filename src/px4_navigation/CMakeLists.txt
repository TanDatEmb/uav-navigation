cmake_minimum_required(VERSION 3.8)
project(px4_navigation)

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
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/local_plan_grid.cpp
    src/virtual_scan.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    px4_common::px4_common
    sensor_msgs::sensor_msgs
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    geometry_msgs
    nav_msgs
    trajectory_msgs
)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(px4_common Eigen3 rclcpp geometry_msgs nav_msgs trajectory_msgs sensor_msgs)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_local_plan_grid test/test_local_plan_grid.cpp)
    target_link_libraries(test_local_plan_grid ${PROJECT_NAME})
    ament_add_gtest(test_virtual_scan test/test_virtual_scan.cpp)
    target_link_libraries(test_virtual_scan ${PROJECT_NAME})
endif()

ament_package()
