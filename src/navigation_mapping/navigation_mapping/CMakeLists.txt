cmake_minimum_required(VERSION 3.16)
project(navigation_mapping LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(rog_map_vendor REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# Explicit source list (no recursive globbing).
set(NAVIGATION_MAPPING_CORE_SOURCES
  src/generation_tracker.cpp
  src/observation_validator.cpp
  src/mapping_point_filter.cpp
  src/rog_map_adapter.cpp
  src/world_model.cpp
  src/mapping_pipeline.cpp
)

add_library(navigation_mapping_core ${NAVIGATION_MAPPING_CORE_SOURCES})
target_compile_features(navigation_mapping_core PUBLIC cxx_std_20)
target_include_directories(navigation_mapping_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(navigation_mapping_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(navigation_mapping_core PUBLIC
  Eigen3::Eigen
  rog_map_vendor::rog_map_vendor
)
ament_target_dependencies(navigation_mapping_core PUBLIC
  builtin_interfaces
  geometry_msgs
)
target_compile_options(navigation_mapping_core PRIVATE -Wall -Wextra)

add_library(navigation_mapping_node_lib
  src/navigation_mapping_node.cpp
)
target_compile_features(navigation_mapping_node_lib PUBLIC cxx_std_20)
target_link_libraries(navigation_mapping_node_lib PUBLIC navigation_mapping_core)
ament_target_dependencies(navigation_mapping_node_lib PUBLIC
  rclcpp
  sensor_msgs
  diagnostic_msgs
  navigation_interfaces
)
target_compile_options(navigation_mapping_node_lib PRIVATE -Wall -Wextra)

add_executable(navigation_mapping_node src/main.cpp)
target_link_libraries(navigation_mapping_node navigation_mapping_node_lib)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS navigation_mapping_core navigation_mapping_node_lib
  EXPORT navigation_mappingTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS navigation_mapping_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  EXPORT navigation_mappingTargets
  FILE navigation_mappingTargets.cmake
  NAMESPACE navigation_mapping::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(navigation_mappingTargets HAS_LIBRARY_TARGET)
# PCL intentionally not re-exported; see rog_map_vendor/CMakeLists.txt for why.
ament_export_dependencies(Eigen3 rog_map_vendor builtin_interfaces geometry_msgs
  rclcpp sensor_msgs diagnostic_msgs navigation_interfaces)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_mapping
    test/test_generation_tracker.cpp
    test/test_observation_validator.cpp
    test/test_mapping_point_filter.cpp
    test/test_mapping_pipeline_generation_reset.cpp
    test/test_rog_map_adapter_geometry.cpp
  )
  target_link_libraries(test_navigation_mapping navigation_mapping_core)
  target_compile_definitions(test_navigation_mapping PRIVATE
    NAVIGATION_MAPPING_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}/test_tmp"
  )
endif()

ament_package()
