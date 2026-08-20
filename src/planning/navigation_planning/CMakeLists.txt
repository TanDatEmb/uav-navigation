cmake_minimum_required(VERSION 3.16)
project(navigation_planning LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(navigation_mapping REQUIRED)

add_library(navigation_planning
  src/a_star.cpp
  src/planner.cpp
  src/trajectory_verifier.cpp
)
target_compile_features(navigation_planning PUBLIC cxx_std_20)
target_include_directories(navigation_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(navigation_planning PUBLIC Eigen3::Eigen)
ament_target_dependencies(navigation_planning PUBLIC navigation_mapping)
target_compile_options(navigation_planning PRIVATE -Wall -Wextra)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS navigation_planning
  EXPORT navigation_planningTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT navigation_planningTargets
  FILE navigation_planningTargets.cmake
  NAMESPACE navigation_planning::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(navigation_planningTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 navigation_mapping)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_planning
    test/test_a_star.cpp
    test/test_bspline_trajectory.cpp
    test/test_planner.cpp
    test/test_rog_world_model_integration.cpp
  )
  target_link_libraries(test_navigation_planning navigation_planning)
  target_compile_definitions(test_navigation_planning PRIVATE
    NAVIGATION_PLANNING_TEST_TMP_DIR="${CMAKE_CURRENT_BINARY_DIR}/test_tmp"
  )
endif()

ament_package()
