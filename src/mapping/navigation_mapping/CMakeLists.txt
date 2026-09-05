cmake_minimum_required(VERSION 3.16)
project(navigation_mapping LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_common REQUIRED)
find_package(navigation_world_model REQUIRED)
find_package(rog_map_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mapping_actor.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  navigation_world_model::navigation_world_model
  rog_map_vendor::rog_map_vendor)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_link_options(${PROJECT_NAME} PRIVATE "LINKER:--exclude-libs,ALL")
endif()
target_compile_definitions(${PROJECT_NAME} PRIVATE NAVIGATION_MAPPING_BUILDING_DLL)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  nav_msgs navigation_common navigation_world_model)

install(FILES
  include/navigation_mapping/mapping_actor.hpp
  include/navigation_mapping/mapping_diagnostics.hpp
  include/navigation_mapping/mapping_observation.hpp
  include/navigation_mapping/mapping_types.hpp
  include/navigation_mapping/traversed_free_space.hpp
  include/navigation_mapping/mapping_worker.hpp
  include/navigation_mapping/observation_accounting.hpp
  include/navigation_mapping/visibility_control.hpp
  include/navigation_mapping/world_snapshot_store.hpp
  DESTINATION include/navigation_mapping)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(navigation_common navigation_world_model)
ament_export_dependencies(nav_msgs)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mapping_worker test/test_mapping_worker.cpp)
  target_include_directories(test_mapping_worker PRIVATE include)
  target_compile_features(test_mapping_worker PRIVATE cxx_std_20)

  ament_add_gtest(test_mapping_actor test/test_mapping_actor.cpp)
  target_compile_features(test_mapping_actor PRIVATE cxx_std_20)
  target_compile_definitions(test_mapping_actor PRIVATE
    ORIGIN_AT_CORNER
    NAVIGATION_MAPPING_PLANNER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/navigation_runtime/config/planner.yaml")
  target_link_libraries(test_mapping_actor ${PROJECT_NAME})
  ament_target_dependencies(test_mapping_actor
    nav_msgs navigation_common navigation_world_model)

  ament_add_gtest(test_mapping_world_model test/test_mapping_world_model.cpp)
  target_compile_features(test_mapping_world_model PRIVATE cxx_std_20)
  target_include_directories(test_mapping_world_model PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_definitions(test_mapping_world_model PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER
    NAVIGATION_PLANNER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/navigation_runtime/config/planner.yaml")
  target_link_libraries(test_mapping_world_model
    navigation_world_model::navigation_world_model
    rog_map_vendor::rog_map_vendor)
  ament_target_dependencies(test_mapping_world_model navigation_world_model)
endif()

ament_package()
