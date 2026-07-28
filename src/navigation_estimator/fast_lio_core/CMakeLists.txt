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

# Sources remain split by algorithm responsibility. CONFIGURE_DEPENDS makes
# additions visible during this from-scratch milestone without maintaining a
# second hand-written inventory beside the documented package layout.
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_library(${PROJECT_NAME} ${CORE_SOURCES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
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
  file(GLOB CORE_TESTS CONFIGURE_DEPENDS test/test_*.cpp)
  foreach(test_source IN LISTS CORE_TESTS)
    get_filename_component(test_name "${test_source}" NAME_WE)
    ament_add_gtest(${test_name} "${test_source}")
    target_link_libraries(${test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 ikfom_vendor)
ament_package()
