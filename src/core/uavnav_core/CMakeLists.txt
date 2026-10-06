cmake_minimum_required(VERSION 3.22)
project(uavnav_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp 0.8 REQUIRED)
find_package(Threads REQUIRED)

# Sanitizer build (make sanitize): "" (off), "thread" or "address". The flags are applied to this
# package's library and tests only (see uavnav_core_sanitize below); they are never added to
# consumers, and `make sanitize` always rebuilds with the option empty afterwards.
set(UAVNAV_SANITIZE "" CACHE STRING "Sanitizer for uavnav_core and its tests: empty, thread or address")
if(NOT UAVNAV_SANITIZE STREQUAL "" AND NOT UAVNAV_SANITIZE STREQUAL "thread" AND NOT UAVNAV_SANITIZE STREQUAL "address")
  message(FATAL_ERROR "UAVNAV_SANITIZE='${UAVNAV_SANITIZE}' is invalid: use empty, 'thread' or 'address'")
endif()
function(uavnav_core_sanitize target)
  if(NOT UAVNAV_SANITIZE STREQUAL "")
    target_compile_options(${target} PRIVATE -fsanitize=${UAVNAV_SANITIZE} -fno-omit-frame-pointer -g)
    target_link_options(${target} PRIVATE -fsanitize=${UAVNAV_SANITIZE})
  endif()
endfunction()

# No ROS (rclcpp) dependency: this library must stay usable outside any node.
# Add library sources to UAVNAV_CORE_SOURCES; add tests below with ament_add_gtest.
set(UAVNAV_CORE_SOURCES
  src/time.cpp
  src/time_convert.cpp
  src/event_recorder.cpp
  src/jsonl_sink.cpp
  src/config.cpp)

add_library(uavnav_core STATIC ${UAVNAV_CORE_SOURCES})
add_library(uavnav_core::uavnav_core ALIAS uavnav_core)
target_include_directories(uavnav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uavnav_core PUBLIC cxx_std_23)
# Strict warnings apply to this package only; they must not leak to consumers.
set(UAVNAV_CORE_STRICT_WARNINGS -Wall -Wextra -Wpedantic -Werror)
target_compile_options(uavnav_core PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
# Static code is linked into shared libraries and nodes of other packages.
set_target_properties(uavnav_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(uavnav_core PUBLIC yaml-cpp::yaml-cpp Threads::Threads)
# A static library is not linked, so only its compile flags matter here; the tests below link it.
uavnav_core_sanitize(uavnav_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS uavnav_core EXPORT export_uavnav_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_uavnav_core HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml-cpp Threads)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_result test/test_result.cpp)
  target_link_libraries(test_result uavnav_core::uavnav_core)
  target_compile_options(test_result PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  uavnav_core_sanitize(test_result)
  ament_add_gtest(test_time test/test_time.cpp)
  target_link_libraries(test_time uavnav_core::uavnav_core)
  target_compile_options(test_time PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  uavnav_core_sanitize(test_time)
  ament_add_gtest(test_event_recorder test/test_event_recorder.cpp)
  target_link_libraries(test_event_recorder uavnav_core::uavnav_core)
  target_compile_options(test_event_recorder PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  uavnav_core_sanitize(test_event_recorder)
  ament_add_gtest(test_golden_jsonl test/test_golden_jsonl.cpp)
  target_link_libraries(test_golden_jsonl uavnav_core::uavnav_core)
  target_compile_options(test_golden_jsonl PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  uavnav_core_sanitize(test_golden_jsonl)
  target_compile_definitions(test_golden_jsonl PRIVATE
    UAVNAV_CORE_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden")
  ament_add_gtest(test_jsonl_sink test/test_jsonl_sink.cpp)
  target_link_libraries(test_jsonl_sink uavnav_core::uavnav_core)
  target_compile_options(test_jsonl_sink PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  uavnav_core_sanitize(test_jsonl_sink)
  ament_add_gtest(test_time_convert test/test_time_convert.cpp)
  target_link_libraries(test_time_convert uavnav_core::uavnav_core)
  target_compile_options(test_time_convert PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  uavnav_core_sanitize(test_time_convert)
  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config uavnav_core::uavnav_core)
  target_compile_options(test_config PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  uavnav_core_sanitize(test_config)
endif()

ament_package()
