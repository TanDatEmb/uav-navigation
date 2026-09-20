cmake_minimum_required(VERSION 3.16)
project(navigation_world_model LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_package()
