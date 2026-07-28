cmake_minimum_required(VERSION 3.16)
project(fast_lio_tools LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(fast_lio_core REQUIRED)

add_library(${PROJECT_NAME}
  src/dataset_reader.cpp
  src/offline_evaluator.cpp
  src/report_writer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME} fast_lio_core)

add_executable(fast_lio_offline_evaluator
  src/fast_lio_offline_evaluator.cpp)
target_link_libraries(fast_lio_offline_evaluator PRIVATE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} fast_lio_offline_evaluator
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fast_lio_core)
ament_package()
