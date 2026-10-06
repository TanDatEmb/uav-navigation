cmake_minimum_required(VERSION 3.22)
project(uavnav_lio_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(uavnav_core REQUIRED)
find_package(fast_lio_core REQUIRED)
find_package(Eigen3 REQUIRED)

# No ROS (rclcpp) dependency: this library must stay usable outside any node.
# Add library sources to UAVNAV_LIO_CORE_SOURCES; add tests below with ament_add_gtest.
set(UAVNAV_LIO_CORE_SOURCES
  src/config.cpp
  src/degeneracy.cpp
  src/lifecycle.cpp
  src/output_predictor.cpp)

add_library(uavnav_lio_core STATIC ${UAVNAV_LIO_CORE_SOURCES})
add_library(uavnav_lio_core::uavnav_lio_core ALIAS uavnav_lio_core)
target_include_directories(uavnav_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uavnav_lio_core PUBLIC cxx_std_23)
# Strict warnings apply to this package only; they must not leak to consumers.
set(UAVNAV_LIO_CORE_STRICT_WARNINGS -Wall -Wextra -Wpedantic -Werror)
target_compile_options(uavnav_lio_core PRIVATE ${UAVNAV_LIO_CORE_STRICT_WARNINGS})
set_target_properties(uavnav_lio_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(uavnav_lio_core PUBLIC
  uavnav_core::uavnav_core fast_lio_core::fast_lio_core Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uavnav_lio_core EXPORT export_uavnav_lio_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_uavnav_lio_core HAS_LIBRARY_TARGET)
ament_export_dependencies(uavnav_core fast_lio_core Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_lio_config test_lifecycle test_degeneracy test_output_predictor)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} uavnav_lio_core::uavnav_lio_core)
    target_compile_options(${test_name} PRIVATE ${UAVNAV_LIO_CORE_STRICT_WARNINGS})
  endforeach()
endif()

ament_package()
