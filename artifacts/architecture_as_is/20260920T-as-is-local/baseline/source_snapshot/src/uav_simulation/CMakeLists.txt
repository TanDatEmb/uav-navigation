cmake_minimum_required(VERSION 3.22)
project(uav_simulation)

find_package(ament_cmake REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(visibility_cloud src/visibility_cloud.cpp)
target_include_directories(visibility_cloud PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(visibility_cloud PUBLIC cxx_std_20)
target_compile_options(visibility_cloud PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(visibility_cloud PUBLIC gz-msgs10::gz-msgs10)
ament_target_dependencies(visibility_cloud PUBLIC sensor_msgs)

add_executable(gz_visibility_bridge src/visibility_bridge.cpp)
target_compile_features(gz_visibility_bridge PRIVATE cxx_std_20)
target_compile_options(gz_visibility_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(gz_visibility_bridge visibility_cloud)
ament_target_dependencies(gz_visibility_bridge rclcpp sensor_msgs)

add_executable(gz_lidar_bridge src/gz_lidar_bridge.cpp)
target_compile_features(gz_lidar_bridge PRIVATE cxx_std_20)
target_compile_options(gz_lidar_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(gz_lidar_bridge
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  ros_gz_bridge::ros_gz_bridge)
ament_target_dependencies(gz_lidar_bridge rclcpp sensor_msgs)

foreach(asset_dir
    bridge
    models
    worlds
)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${asset_dir}")
    install(
      DIRECTORY "${asset_dir}"
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_px4_mid360_assets.py")
    ament_add_pytest_test(
      test_px4_mid360_assets
      test/test_px4_mid360_assets.py
    )
  endif()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_visibility_cloud test/test_visibility_cloud.cpp)
  target_link_libraries(test_visibility_cloud visibility_cloud)
endif()

install(TARGETS visibility_cloud gz_visibility_bridge gz_lidar_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
