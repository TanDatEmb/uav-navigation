cmake_minimum_required(VERSION 3.16)
project(rog_map_vendor LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
# uav-navigation local modification: PCL's "io" COMPONENTS entry pulls in a
# VTK/MPI CMake config path that is broken on this Ubuntu 24.04 image
# (VTK::mpi references MPI::MPI_C without find_package(MPI) ever running).
# rog_map.cpp only calls pcl::io::loadPCDFile() for the optional (P1: OFF)
# load_pcd_en feature, so link libpcl_io directly instead of requesting the
# "io" PCL component.
find_library(PCL_IO_LIBRARY NAMES pcl_io REQUIRED)
find_library(PCL_COMMON_LIBRARY NAMES pcl_common REQUIRED)
# uav-navigation local modification (see UPSTREAM.md): link the system
# libyaml-cpp-dev/libfmt-dev packages instead of vendoring fmt's ~13 headers
# and relying on catkin/ament yaml-cpp wrapper packages. Both ship first-class
# CMake config files on Ubuntu 24.04 / ROS 2 Jazzy.
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

# Upstream defines the voxel/raycast origin convention at compile time. This
# repository only ever uses ORIGIN_AT_CORNER; ORIGIN_AT_CENTER is untested and
# intentionally not wired up.
set(ROG_MAP_VENDOR_DEFINITIONS
  -DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
  -DORIGIN_AT_CORNER
)

# Explicit source list (no recursive globbing, per repository convention).
set(ROG_MAP_VENDOR_SOURCES
  src/rog_map_core/raycaster.cpp
  src/rog_map/sliding_map.cpp
  src/rog_map/counter_map.cpp
  src/rog_map/inf_map.cpp
  src/rog_map/esdf_map.cpp
  src/rog_map/prob_map.cpp
  src/rog_map/rog_map.cpp
)

add_library(rog_map_vendor STATIC ${ROG_MAP_VENDOR_SOURCES})
set_target_properties(rog_map_vendor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(rog_map_vendor PUBLIC cxx_std_20)
target_compile_definitions(rog_map_vendor PUBLIC ${ROG_MAP_VENDOR_DEFINITIONS})
target_include_directories(rog_map_vendor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(rog_map_vendor SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(rog_map_vendor PUBLIC
  Eigen3::Eigen
  ${PCL_COMMON_LIBRARY}
  ${PCL_IO_LIBRARY}
  yaml-cpp::yaml-cpp
  fmt::fmt
)

# Local build-quality flags scoped to this target only (never global).
target_compile_options(rog_map_vendor PRIVATE -Wall)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS rog_map_vendor
  EXPORT rog_map_vendorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  EXPORT rog_map_vendorTargets
  FILE rog_map_vendorTargets.cmake
  NAMESPACE rog_map_vendor::
  DESTINATION share/${PROJECT_NAME}/cmake
)
ament_export_targets(rog_map_vendorTargets HAS_LIBRARY_TARGET)
# uav-navigation local modification: PCL and pcl_conversions are intentionally
# NOT re-exported here. rog_map_vendorTargets.cmake already embeds their
# resolved library paths and include directories on the rog_map_vendor target
# itself (via ament_target_dependencies above), so downstream packages do not
# need to find_package() either again. Re-exporting either would make every
# consumer hit the broken VTK::mpi/MPI::MPI_C PCLConfig.cmake path documented
# above (pcl_conversions' own CMake config performs an unqualified
# find_package(PCL), which pulls the same VTK component set).
ament_export_dependencies(Eigen3 yaml-cpp fmt)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rog_map_vendor
    test/test_rog_map_smoke.cpp
    test/test_rog_map_lifecycle.cpp
    test/test_raycaster_equivalence.cpp
    test/test_planning_grid_export.cpp
    test/test_yaml_loader.cpp
  )
  target_link_libraries(test_rog_map_vendor rog_map_vendor)
  target_compile_definitions(test_rog_map_vendor PRIVATE
    ROG_MAP_VENDOR_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
endif()

ament_package()
