cmake_minimum_required(VERSION 3.16)
project(super_planner_vendor LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rog_map_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(SUPER_PLANNER_SOURCES
  src/super_core/astar.cpp
  src/super_core/ciri.cpp
  src/super_core/corridor_generator.cpp
  src/super_core/super_planner.cpp
  src/traj_opt/backup_traj_optimizer_s4.cpp
  src/traj_opt/exp_traj_optimizer_s4.cpp
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
  src/utils/waypoint_trajectory_optimizer.cpp
)

add_library(${PROJECT_NAME} STATIC ${SUPER_PLANNER_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_compile_definitions(${PROJECT_NAME} PUBLIC
  USE_ROS2
  ORIGIN_AT_CORNER
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  fmt::fmt
  rog_map_vendor::rog_map_vendor
  yaml-cpp::yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC std_msgs)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter)

install(DIRECTORY include/ DESTINATION include)
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
ament_export_dependencies(Eigen3 fmt rog_map_vendor std_msgs yaml-cpp)
ament_export_include_directories(include)
ament_package()

ament_add_gtest(test_trajectory test/test_trajectory.cpp)
target_link_libraries(test_trajectory ${PROJECT_NAME})

ament_add_gtest(test_super_config test/test_super_config.cpp)
target_compile_definitions(test_super_config PRIVATE
  SUPER_PRODUCT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../../runtime/navigation_runtime/config/super_planner.yaml")
target_link_libraries(test_super_config ${PROJECT_NAME})
