cmake_minimum_required(VERSION 3.16)
project(navigation_planning_backend LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(navigation_world_model REQUIRED)
find_package(navigation_planning REQUIRED)
find_package(navigation_mission REQUIRED)
find_package(rog_map_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(NAVIGATION_PLANNING_BACKEND_SOURCES
  src/planner_core/astar.cpp
  src/planner_core/ciri.cpp
  src/planner_core/corridor_generator.cpp
  src/planner_core/planner.cpp
  src/planner_core/route_yaw_reference.cpp
  src/planner_facade.cpp
  src/traj_opt/backup_trajectory_optimizer.cpp
  src/traj_opt/nominal_trajectory_optimizer.cpp
  src/traj_opt/yaw_traj_opt.cpp
  src/utils/banded_system.cpp
  src/utils/ellipsoid.cpp
  src/utils/geometry_utils.cpp
  src/utils/lbfgs.cpp
  src/utils/minco.cpp
  src/utils/mvie.cpp
  src/utils/optimization_utils.cpp
  src/utils/piece.cpp
  src/utils/polynomial_interpolation.cpp
  src/utils/polytope.cpp
  src/utils/quickhull.cpp
  src/utils/root_finder.cpp
  src/utils/sdlp.cpp
  src/utils/trajectory.cpp
)

add_library(${PROJECT_NAME} STATIC ${NAVIGATION_PLANNING_BACKEND_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  USE_ROS2
  ORIGIN_AT_CORNER
)
target_include_directories(${PROJECT_NAME} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  navigation_planning::navigation_planning
  navigation_mission::navigation_mission
  navigation_world_model::navigation_world_model
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  fmt::fmt
  rog_map_vendor::rog_map_vendor
  yaml-cpp::yaml-cpp
)
if(std_msgs_TARGETS)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${std_msgs_TARGETS})
else()
  target_include_directories(${PROJECT_NAME} PRIVATE ${std_msgs_INCLUDE_DIRS})
endif()
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter)

install(FILES include/navigation_planning_backend/planner_facade.hpp
  DESTINATION include/navigation_planning_backend)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 fmt navigation_mission navigation_planning navigation_world_model rog_map_vendor std_msgs yaml-cpp)
ament_export_include_directories(include)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(navigation_mapping REQUIRED)

  function(configure_backend_test target)
    target_link_libraries(${target}
      fmt::fmt
      rog_map_vendor::rog_map_vendor
      yaml-cpp::yaml-cpp)
    target_include_directories(${target} PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(${target}
      navigation_mission navigation_planning navigation_world_model rog_map_vendor std_msgs)
  endfunction()

  ament_add_gtest(test_trajectory test/test_trajectory.cpp)
  target_compile_definitions(test_trajectory PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER
    NAVIGATION_PLANNING_BACKEND_TESTING)
  target_link_libraries(test_trajectory ${PROJECT_NAME})
  configure_backend_test(test_trajectory)

  ament_add_gtest(test_planner_config test/test_planner_config.cpp)
  target_compile_definitions(test_planner_config PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER
    PLANNER_PRODUCT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/navigation_runtime/config/planner.yaml")
  target_link_libraries(test_planner_config ${PROJECT_NAME})
  configure_backend_test(test_planner_config)

  ament_add_gtest(test_planner_facade test/test_planner_facade.cpp)
  target_compile_definitions(test_planner_facade PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER
    PLANNER_FACADE_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/navigation_runtime/config/planner.yaml")
  target_link_libraries(test_planner_facade ${PROJECT_NAME})
  target_link_libraries(test_planner_facade navigation_mapping::navigation_mapping)
  configure_backend_test(test_planner_facade)

  ament_add_gtest(test_planner_runtime_context
    test/test_planner_runtime_context.cpp)
  target_compile_definitions(test_planner_runtime_context PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER)
  target_link_libraries(test_planner_runtime_context ${PROJECT_NAME})
  configure_backend_test(test_planner_runtime_context)

  ament_add_gtest(test_route_yaw_reference
    test/test_route_yaw_reference.cpp)
  target_compile_definitions(test_route_yaw_reference PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER)
  target_link_libraries(test_route_yaw_reference ${PROJECT_NAME})
  configure_backend_test(test_route_yaw_reference)

  ament_add_gtest(test_route_backbone
    test/test_route_backbone.cpp)
  target_compile_definitions(test_route_backbone PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER)
  target_link_libraries(test_route_backbone ${PROJECT_NAME})
  configure_backend_test(test_route_backbone)

  ament_add_gtest(test_corridor_bezier_seed
    test/test_corridor_bezier_seed.cpp)
  target_compile_definitions(test_corridor_bezier_seed PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER
    PLANNER_CORRIDOR_BEZIER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/navigation_runtime/config/planner.yaml")
  target_link_libraries(test_corridor_bezier_seed ${PROJECT_NAME})
  configure_backend_test(test_corridor_bezier_seed)

  ament_add_gtest(test_exp_optimizer_seed
    test/test_exp_optimizer_seed.cpp)
  target_compile_definitions(test_exp_optimizer_seed PRIVATE
    USE_ROS2
    ORIGIN_AT_CORNER
    PLANNER_EXP_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/navigation_runtime/config/planner.yaml")
  target_link_libraries(test_exp_optimizer_seed ${PROJECT_NAME})
  configure_backend_test(test_exp_optimizer_seed)
endif()

ament_package()
