cmake_minimum_required(VERSION 3.8)
project(px4_common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Header-only library with a single translation unit so ament can export and
# install the headers correctly for downstream packages.
add_library(${PROJECT_NAME} src/px4_common.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC geometry_msgs rclcpp)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 geometry_msgs rclcpp)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_grid test/test_grid.cpp)
    target_link_libraries(test_grid ${PROJECT_NAME})
    ament_add_gtest(test_transforms test/test_transforms.cpp)
    target_link_libraries(test_transforms ${PROJECT_NAME})
    ament_add_gtest(test_types test/test_types.cpp)
    target_link_libraries(test_types ${PROJECT_NAME})
    ament_add_gtest(test_voxel_types test/test_voxel_types.cpp)
    target_link_libraries(test_voxel_types ${PROJECT_NAME})
    ament_add_gtest(test_voxel_map_interface test/test_voxel_map_interface.cpp)
    target_link_libraries(test_voxel_map_interface ${PROJECT_NAME})
    ament_add_gtest(test_pose_buffer test/test_pose_buffer.cpp)
    target_link_libraries(test_pose_buffer ${PROJECT_NAME})
endif()

ament_package()
