cmake_minimum_required(VERSION 3.8)
project(px4_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} INTERFACE geometry_msgs rclcpp)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_math_utils test/test_math_utils.cpp)
    target_link_libraries(test_math_utils ${PROJECT_NAME})
    ament_add_gtest(test_transforms test/test_transforms.cpp)
    target_link_libraries(test_transforms ${PROJECT_NAME})
    ament_add_gtest(test_grid test/test_grid.cpp)
    target_link_libraries(test_grid ${PROJECT_NAME})
    ament_add_gtest(test_types test/test_types.cpp)
    target_link_libraries(test_types ${PROJECT_NAME})
endif()

ament_package()
