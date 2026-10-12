cmake_minimum_required(VERSION 3.22)
project(uavnav_lio_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(uavnav_core REQUIRED)
find_package(fast_lio_core REQUIRED)
find_package(Eigen3 REQUIRED)

# Sanitizer build (make sanitize, D28): "" (off), "thread" or "address", validated like uavnav_core's. The flags
# are applied PRIVATE to this package's library and to every test (uavnav_lio_core_sanitize below); they never
# reach consumers, and `make sanitize` always rebuilds with the option empty afterwards. Dependencies keep their
# own build: uavnav_core is instrumented by its own entry in the `make sanitize` loop; fast_lio_core (the reused
# math, called only on the backend thread) and uavnav_px4_bridge are not instrumented. With "thread",
# UAVNAV_LIO_TSAN_BUILD sets limits::kIcpThreads to 1: libgomp is not TSan-instrumented (GCC PR 55561).
set(UAVNAV_SANITIZE "" CACHE STRING "Sanitizer for uavnav_lio_core and its tests: empty, thread or address")
if(NOT UAVNAV_SANITIZE STREQUAL "" AND NOT UAVNAV_SANITIZE STREQUAL "thread" AND NOT UAVNAV_SANITIZE STREQUAL "address")
  message(FATAL_ERROR "UAVNAV_SANITIZE='${UAVNAV_SANITIZE}' is invalid: use empty, 'thread' or 'address'")
endif()
function(uavnav_lio_core_sanitize target)
  if(NOT UAVNAV_SANITIZE STREQUAL "")
    target_compile_options(${target} PRIVATE -fsanitize=${UAVNAV_SANITIZE} -fno-omit-frame-pointer -g)
    target_link_options(${target} PRIVATE -fsanitize=${UAVNAV_SANITIZE})
  endif()
  if(UAVNAV_SANITIZE STREQUAL "thread")
    target_compile_definitions(${target} PRIVATE UAVNAV_LIO_TSAN_BUILD)
  endif()
endfunction()

# No ROS (rclcpp) dependency: this library must stay usable outside any node.
# Add library sources to UAVNAV_LIO_CORE_SOURCES; add tests below with ament_add_gtest.
set(UAVNAV_LIO_CORE_SOURCES
  src/backend.cpp
  src/backend_math.cpp
  src/config.cpp
  src/degeneracy.cpp
  src/frontend.cpp
  src/lifecycle.cpp
  src/output_predictor.cpp
  src/pipeline.cpp
  src/runner.cpp)

add_library(uavnav_lio_core STATIC ${UAVNAV_LIO_CORE_SOURCES})
add_library(uavnav_lio_core::uavnav_lio_core ALIAS uavnav_lio_core)
target_include_directories(uavnav_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uavnav_lio_core PUBLIC cxx_std_23)
# Strict warnings apply to this package only; they must not leak to consumers.
set(UAVNAV_LIO_CORE_STRICT_WARNINGS -Wall -Wextra -Wpedantic -Werror)
target_compile_options(uavnav_lio_core PRIVATE ${UAVNAV_LIO_CORE_STRICT_WARNINGS})
set_target_properties(uavnav_lio_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(uavnav_lio_core PUBLIC
  uavnav_core::uavnav_core fast_lio_core::fast_lio_core Eigen3::Eigen)
# A static library is not linked, so only its compile flags matter here; the tests below link it.
uavnav_lio_core_sanitize(uavnav_lio_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uavnav_lio_core EXPORT export_uavnav_lio_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_uavnav_lio_core HAS_LIBRARY_TARGET)
ament_export_dependencies(uavnav_core fast_lio_core Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The tracked default parameter file, found from this package's source dir (portable, no install step).
  get_filename_component(UAVNAV_LIO_DEFAULT_YAML "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/lio/sim.yaml" ABSOLUTE)
  foreach(test_name test_lio_config test_lifecycle test_degeneracy test_output_predictor test_pipeline
      test_backend test_backend_boundary test_frontend test_runner)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 300)
    target_link_libraries(${test_name} uavnav_lio_core::uavnav_lio_core)
    target_compile_options(${test_name} PRIVATE ${UAVNAV_LIO_CORE_STRICT_WARNINGS})
    uavnav_lio_core_sanitize(${test_name})
    target_compile_definitions(${test_name} PRIVATE UAVNAV_LIO_DEFAULT_YAML="${UAVNAV_LIO_DEFAULT_YAML}"
      UAVNAV_LIO_CORE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endforeach()
endif()

ament_package()
