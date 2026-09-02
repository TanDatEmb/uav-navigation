cmake_minimum_required(VERSION 3.16)
project(navigation_planning LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(navigation_common REQUIRED)
find_package(navigation_mission REQUIRED)
find_package(navigation_world_model REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} INTERFACE navigation_common::navigation_common)
target_link_libraries(${PROJECT_NAME} INTERFACE navigation_mission::navigation_mission)
target_link_libraries(${PROJECT_NAME} INTERFACE navigation_world_model::navigation_world_model)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(
  Eigen3
  builtin_interfaces
  navigation_common
  navigation_mission
  navigation_world_model)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planning_contracts test/test_planning_contracts.cpp)
  target_compile_features(test_planning_contracts PRIVATE cxx_std_20)
  target_link_libraries(test_planning_contracts ${PROJECT_NAME})
  ament_target_dependencies(test_planning_contracts builtin_interfaces)
endif()

ament_package()
