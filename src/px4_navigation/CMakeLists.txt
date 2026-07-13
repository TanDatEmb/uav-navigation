cmake_minimum_required(VERSION 3.8)
project(px4_navigation)
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
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
add_library(${PROJECT_NAME} SHARED
    src/local_plan_grid.cpp
    src/virtual_scan.cpp
    src/a_star_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    px4_navigation_common::px4_navigation_common
    sensor_msgs::sensor_msgs
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
    px4_ros2_utils
    rclcpp
    geometry_msgs
    nav_msgs
    trajectory_msgs
    sensor_msgs
)
# ── Obstacle perception node ───────────────────────────────────────────────
add_executable(obstacle_perception
    src/obstacle_perception.cpp
    src/obstacle_perception_main.cpp)
target_include_directories(obstacle_perception PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(obstacle_perception
    ${PROJECT_NAME}
    Eigen3::Eigen
    px4_navigation_common::px4_navigation_common
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    px4_msgs::px4_msgs
    visualization_msgs::visualization_msgs
)
ament_target_dependencies(obstacle_perception px4_ros2_utils)
install(TARGETS obstacle_perception
    DESTINATION lib/${PROJECT_NAME}
)
# ── Obstacle distance visualizer node ───────────────────────────
add_executable(obstacle_distance_visualizer_node
    src/obstacle_distance_visualizer.cpp
    src/obstacle_distance_visualizer_main.cpp)
target_include_directories(obstacle_distance_visualizer_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(obstacle_distance_visualizer_node
    px4_navigation_common::px4_navigation_common
    rclcpp::rclcpp
    px4_msgs::px4_msgs
    visualization_msgs::visualization_msgs
)
ament_target_dependencies(obstacle_distance_visualizer_node px4_ros2_utils)
install(TARGETS obstacle_distance_visualizer_node
    DESTINATION lib/${PROJECT_NAME}
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
# Install config and launch files so launch can find them via get_package_share_directory.
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(px4_navigation_common px4_ros2_utils px4_msgs Eigen3 rclcpp geometry_msgs nav_msgs trajectory_msgs sensor_msgs visualization_msgs)
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_local_plan_grid test/test_local_plan_grid.cpp)
    target_link_libraries(test_local_plan_grid ${PROJECT_NAME})
    ament_add_gtest(test_virtual_scan test/test_virtual_scan.cpp)
    target_link_libraries(test_virtual_scan ${PROJECT_NAME})
    ament_add_gtest(test_a_star_planner test/test_a_star_planner.cpp)
    target_link_libraries(test_a_star_planner ${PROJECT_NAME})
endif()
ament_package()
