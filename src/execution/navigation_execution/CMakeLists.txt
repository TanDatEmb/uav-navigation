cmake_minimum_required(VERSION 3.16)
project(navigation_execution LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(navigation_common REQUIRED)
find_package(navigation_contracts REQUIRED)
find_package(navigation_planning REQUIRED)
find_package(navigation_world_model REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  navigation_common::navigation_common
  navigation_planning::navigation_planning
  navigation_world_model::navigation_world_model
)
target_include_directories(${PROJECT_NAME} INTERFACE ${navigation_contracts_INCLUDE_DIRS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  navigation_common
  navigation_contracts
  navigation_planning
  navigation_world_model)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(navigation_mission REQUIRED)
  ament_add_gtest(test_execution_state test/test_timestamp_freshness.cpp)
  target_include_directories(test_execution_state PRIVATE
    include ${navigation_contracts_INCLUDE_DIRS})
  target_compile_features(test_execution_state PRIVATE cxx_std_20)
  ament_target_dependencies(test_execution_state navigation_contracts)

  ament_add_gtest(test_execution_authority test/test_execution_authority.cpp)
  target_compile_features(test_execution_authority PRIVATE cxx_std_20)
  target_link_libraries(test_execution_authority ${PROJECT_NAME})
  ament_target_dependencies(test_execution_authority
    builtin_interfaces navigation_common navigation_planning navigation_world_model)

  ament_add_gtest(test_certified_continuation
    test/test_certified_continuation.cpp)
  target_include_directories(test_certified_continuation PRIVATE include)
  target_link_libraries(test_certified_continuation
    navigation_mission::navigation_mission)
  ament_target_dependencies(test_certified_continuation
    navigation_planning)

  ament_add_gtest(test_world_temporal_assessment
    test/test_world_temporal_assessment.cpp)
  target_include_directories(test_world_temporal_assessment PRIVATE include)
  ament_target_dependencies(test_world_temporal_assessment
    navigation_world_model)
endif()

ament_package()
