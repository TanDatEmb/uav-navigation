cmake_minimum_required(VERSION 3.16)
project(fast_lio_core LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ikfom_vendor REQUIRED)
find_package(ikd_tree_vendor REQUIRED)
find_package(OpenMP REQUIRED)

set(CORE_SOURCES
  src/deskew/scan_deskewer.cpp
  src/estimation/ikfom_estimator.cpp
  src/estimation/ikfom_state.cpp
  src/estimation/imu_trajectory.cpp
  src/estimation/manifold_state.cpp
  src/geometry/pose_interpolator.cpp
  src/geometry/rigid_transform.cpp
  src/geometry/transform_utils.cpp
  src/initialization/imu_initializer.cpp
  src/initialization/initial_state_prior.cpp
  src/initialization/initial_state_prior_applicator.cpp
  src/mapping/ikd_tree_registration_map.cpp
  src/mapping/local_map_manager.cpp
  src/mapping/map_insertion_policy.cpp
  src/navigation/angular_velocity_resolver.cpp
  src/navigation/base_link_covariance_projector.cpp
  src/navigation/base_link_state_converter.cpp
  src/pipeline/estimator_diagnostics.cpp
  src/pipeline/fast_lio_pipeline.cpp
  src/preprocessing/point_cloud_preprocessor.cpp
  src/preprocessing/point_filter.cpp
  src/preprocessing/voxel_filter.cpp
  src/propagation/imu_state_propagator.cpp
  src/registration/linearized_measurement.cpp
  src/registration/plane_estimator.cpp
  src/registration/residual_builder.cpp
  src/registration/residual_gate.cpp
  src/sensor/imu_sample.cpp
  src/sensor/lidar_scan.cpp
  src/synchronization/measurement_buffer.cpp
  src/synchronization/measurement_synchronizer.cpp
  src/time/duration.cpp
  src/time/timestamp.cpp
  src/time/timestamp_validator.cpp)
add_library(${PROJECT_NAME} ${CORE_SOURCES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen ikd_tree_vendor::ikd_tree_vendor OpenMP::OpenMP_CXX)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME} ikfom_vendor)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(CORE_TESTS
    test/test_angular_velocity_resolver.cpp
    test/test_base_link_covariance_projector.cpp
    test/test_base_link_state_converter.cpp
    test/test_duration.cpp
    test/test_fast_lio_pipeline.cpp
    test/test_frame_conventions.cpp
    test/test_ikfom_compact_equivalence.cpp
    test/test_ikfom_estimator.cpp
    test/test_ikfom_incremental_propagation.cpp
    test/test_ikfom_jacobians.cpp
    test/test_imu_initializer.cpp
    test/test_imu_state_propagator.cpp
    test/test_initial_state_prior.cpp
    test/test_initial_state_prior_pipeline.cpp
    test/test_measurement_buffer.cpp
    test/test_measurement_synchronizer.cpp
    test/test_plane_estimator.cpp
    test/test_point_filter.cpp
    test/test_pose_interpolator.cpp
    test/test_registration_map.cpp
    test/test_residual_builder.cpp
    test/test_rigid_transform.cpp
    test/test_scan_deskewer.cpp
    test/test_timestamp.cpp)
  foreach(test_source IN LISTS CORE_TESTS)
    get_filename_component(test_name "${test_source}" NAME_WE)
    ament_add_gtest(${test_name} "${test_source}")
    target_link_libraries(${test_name} ${PROJECT_NAME})
    target_include_directories(${test_name} PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/test")
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 ikfom_vendor ikd_tree_vendor)
ament_package()
