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

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(builtin_interfaces navigation_common navigation_planning navigation_world_model)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(navigation_contracts REQUIRED)
  ament_add_gtest(test_execution_state test/test_timestamp_freshness.cpp)
  target_include_directories(test_execution_state PRIVATE
    include ${navigation_contracts_INCLUDE_DIRS})
  target_compile_features(test_execution_state PRIVATE cxx_std_20)
  ament_target_dependencies(test_execution_state navigation_contracts)

  ament_add_gtest(test_committed_bundle_store test/test_committed_bundle_store.cpp)
  target_compile_features(test_committed_bundle_store PRIVATE cxx_std_20)
  target_link_libraries(test_committed_bundle_store ${PROJECT_NAME})
  ament_target_dependencies(test_committed_bundle_store
    builtin_interfaces navigation_common navigation_planning navigation_world_model)
endif()

ament_package()
