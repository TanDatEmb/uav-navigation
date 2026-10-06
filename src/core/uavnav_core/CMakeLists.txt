cmake_minimum_required(VERSION 3.22)
project(uavnav_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# No ROS (rclcpp) dependency: this library must stay usable outside any node.
# Add library sources to UAVNAV_CORE_SOURCES; add tests below with ament_add_gtest.
set(UAVNAV_CORE_SOURCES
  src/time.cpp
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
  ament_add_gtest(test_time test/test_time.cpp)
  target_link_libraries(test_time uavnav_core::uavnav_core)
  target_compile_options(test_time PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  ament_add_gtest(test_event_recorder test/test_event_recorder.cpp)
  target_link_libraries(test_event_recorder uavnav_core::uavnav_core)
  target_compile_options(test_event_recorder PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  ament_add_gtest(test_golden_jsonl test/test_golden_jsonl.cpp)
  target_link_libraries(test_golden_jsonl uavnav_core::uavnav_core)
  target_compile_options(test_golden_jsonl PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  target_compile_definitions(test_golden_jsonl PRIVATE
    UAVNAV_CORE_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/golden")
  ament_add_gtest(test_jsonl_sink test/test_jsonl_sink.cpp)
  target_link_libraries(test_jsonl_sink uavnav_core::uavnav_core)
  target_compile_options(test_jsonl_sink PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config uavnav_core::uavnav_core)
  target_compile_options(test_config PRIVATE ${UAVNAV_CORE_STRICT_WARNINGS})
endif()

ament_package()
