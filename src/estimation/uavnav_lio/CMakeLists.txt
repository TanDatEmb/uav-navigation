cmake_minimum_required(VERSION 3.22)
project(uavnav_lio LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(uavnav_core REQUIRED)
find_package(uavnav_lio_core REQUIRED)
find_package(Eigen3 REQUIRED)

# The lio process (D33/O19): a ROS-free ingest target (uavnav_lio_ingest, this section) and a node target
# (S1b Task 8, appended below the ingest target). The ingest target links no rclcpp and no message package.
#
# Sanitizer build (make sanitize, D28): "" (off), "thread" or "address", validated like uavnav_lio_core's. The
# flags are applied PRIVATE to the ingest library and its tests only (uavnav_lio_sanitize below); uavnav_core and
# uavnav_lio_core are instrumented by their own entries in the `make sanitize` loop. With "thread",
# UAVNAV_LIO_TSAN_BUILD is defined here too: uavnav/lio/limits.hpp (kIcpThreads) is included by this package's
# translation units, and its value must be the same in every translation unit of one program as in
# uavnav_lio_core's (ODR). Targets that need ROS are not built under UAVNAV_SANITIZE (Task 8).
set(UAVNAV_SANITIZE "" CACHE STRING "Sanitizer for uavnav_lio_ingest and its tests: empty, thread or address")
if(NOT UAVNAV_SANITIZE STREQUAL "" AND NOT UAVNAV_SANITIZE STREQUAL "thread" AND NOT UAVNAV_SANITIZE STREQUAL "address")
  message(FATAL_ERROR "UAVNAV_SANITIZE='${UAVNAV_SANITIZE}' is invalid: use empty, 'thread' or 'address'")
endif()
function(uavnav_lio_sanitize target)
  if(NOT UAVNAV_SANITIZE STREQUAL "")
    target_compile_options(${target} PRIVATE -fsanitize=${UAVNAV_SANITIZE} -fno-omit-frame-pointer -g)
    target_link_options(${target} PRIVATE -fsanitize=${UAVNAV_SANITIZE})
  endif()
  if(UAVNAV_SANITIZE STREQUAL "thread")
    target_compile_definitions(${target} PRIVATE UAVNAV_LIO_TSAN_BUILD)
  endif()
endfunction()

# Strict warnings apply to this package only; they must not leak to consumers.
set(UAVNAV_LIO_STRICT_WARNINGS -Wall -Wextra -Wpedantic -Werror)

# --- uavnav_lio_ingest: ROS-free (no rclcpp, no messages, no clock read but SharedRosTime's steady stamp) ----
add_library(uavnav_lio_ingest STATIC src/node_config.cpp src/ingest.cpp)
add_library(uavnav_lio::uavnav_lio_ingest ALIAS uavnav_lio_ingest)
target_include_directories(uavnav_lio_ingest PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uavnav_lio_ingest PUBLIC cxx_std_23)
target_compile_options(uavnav_lio_ingest PRIVATE ${UAVNAV_LIO_STRICT_WARNINGS})
set_target_properties(uavnav_lio_ingest PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(uavnav_lio_ingest PUBLIC
  uavnav_lio_core::uavnav_lio_core uavnav_core::uavnav_core Eigen3::Eigen)
# A static library is not linked, so only its compile flags matter here; the tests below link it.
uavnav_lio_sanitize(uavnav_lio_ingest)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uavnav_lio_ingest EXPORT export_uavnav_lio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_uavnav_lio HAS_LIBRARY_TARGET)
ament_export_dependencies(uavnav_core uavnav_lio_core Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The tracked default parameter files, found from this package's source dir (portable, no install step).
  get_filename_component(UAVNAV_LIO_DEFAULT_YAML "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/lio/sim.yaml" ABSOLUTE)
  get_filename_component(UAVNAV_LIO_NODE_YAML "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/lio/node_sim.yaml" ABSOLUTE)
  foreach(test_name test_node_config test_ingest)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 300)
    target_link_libraries(${test_name} uavnav_lio::uavnav_lio_ingest)
    target_compile_options(${test_name} PRIVATE ${UAVNAV_LIO_STRICT_WARNINGS})
    uavnav_lio_sanitize(${test_name})
    target_compile_definitions(${test_name} PRIVATE UAVNAV_LIO_DEFAULT_YAML="${UAVNAV_LIO_DEFAULT_YAML}"
      UAVNAV_LIO_NODE_YAML="${UAVNAV_LIO_NODE_YAML}")
  endforeach()
endif()

ament_package()
