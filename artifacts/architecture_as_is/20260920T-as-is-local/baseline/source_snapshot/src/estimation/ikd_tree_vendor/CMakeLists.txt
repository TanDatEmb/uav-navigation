cmake_minimum_required(VERSION 3.22)
project(ikd_tree_vendor)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

set(IKD_TREE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/ikd-Tree")
if(NOT EXISTS "${IKD_TREE_ROOT}/ikd-Tree/ikd_Tree.h" OR
   NOT EXISTS "${IKD_TREE_ROOT}/ikd-Tree/ikd_Tree.cpp")
  message(FATAL_ERROR
    "Pinned ikd-Tree source is missing. Restore vendor/ikd-Tree at the commit "
    "recorded in UPSTREAM.md; network downloads during build are forbidden.")
endif()

add_library(ikd_tree_vendor
  "${IKD_TREE_ROOT}/ikd-Tree/ikd_Tree.cpp")
add_library(ikd_tree_vendor::ikd_tree_vendor ALIAS ikd_tree_vendor)
target_compile_features(ikd_tree_vendor PUBLIC cxx_std_20)
target_include_directories(ikd_tree_vendor PUBLIC
  "$<BUILD_INTERFACE:${IKD_TREE_ROOT}/ikd-Tree>"
  "$<INSTALL_INTERFACE:include/ikd_tree_vendor>")
target_link_libraries(ikd_tree_vendor PUBLIC ${PCL_LIBRARIES} pthread)
target_compile_options(ikd_tree_vendor PRIVATE
  -Wno-unused-variable -Wno-sign-compare)

install(TARGETS ikd_tree_vendor
  EXPORT export_ikd_tree_vendor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES "${IKD_TREE_ROOT}/ikd-Tree/ikd_Tree.h"
  DESTINATION include/ikd_tree_vendor)
install(DIRECTORY LICENSES DESTINATION share/${PROJECT_NAME})
install(FILES UPSTREAM.md PATCHES.md
  "${IKD_TREE_ROOT}/LICENSE" "${IKD_TREE_ROOT}/README.md"
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ikd_tree_upstream_smoke
    test/test_ikd_tree_upstream_smoke.cpp)
  target_link_libraries(test_ikd_tree_upstream_smoke ikd_tree_vendor)
endif()

ament_export_targets(export_ikd_tree_vendor HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL)
ament_package()
