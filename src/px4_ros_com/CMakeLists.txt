cmake_minimum_required(VERSION 3.8)
project(px4_ros_com)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(px4_common REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    px4_common
    px4_msgs
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
)

add_library(frame_transforms INTERFACE)
target_include_directories(frame_transforms INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(frame_transforms INTERFACE px4_common::px4_common)
ament_target_dependencies(frame_transforms INTERFACE ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    TARGETS frame_transforms
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_frame_transforms test/test_frame_transforms.cpp)
    target_link_libraries(test_frame_transforms frame_transforms)
    ament_target_dependencies(test_frame_transforms ${THIS_PACKAGE_INCLUDE_DEPENDS})
    ament_add_gtest(test_topic_helpers test/test_topic_helpers.cpp)
    target_link_libraries(test_topic_helpers frame_transforms)
    ament_target_dependencies(test_topic_helpers ${THIS_PACKAGE_INCLUDE_DEPENDS})
    ament_add_gtest(test_time_sync test/test_time_sync.cpp)
    target_link_libraries(test_time_sync frame_transforms)
    ament_target_dependencies(test_time_sync ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

ament_package()
