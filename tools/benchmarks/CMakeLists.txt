cmake_minimum_required(VERSION 3.22)
project(m1_ikd_tree_benchmark LANGUAGES CXX)

find_package(PCL REQUIRED COMPONENTS common)

set(IKD_TREE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../src/navigation_estimator/ikd_tree_vendor/vendor/ikd-Tree")
if(NOT EXISTS "${IKD_TREE_ROOT}/ikd-Tree/ikd_Tree.h" OR
   NOT EXISTS "${IKD_TREE_ROOT}/ikd-Tree/ikd_Tree.cpp")
  message(FATAL_ERROR "Pinned ikd-Tree source is missing; benchmark cannot use a substitute backend.")
endif()

add_executable(ikd_tree_benchmark
  ikd_tree_benchmark.cpp
  "${IKD_TREE_ROOT}/ikd-Tree/ikd_Tree.cpp")
target_compile_features(ikd_tree_benchmark PRIVATE cxx_std_20)
target_include_directories(ikd_tree_benchmark PRIVATE "${IKD_TREE_ROOT}/ikd-Tree")
target_link_libraries(ikd_tree_benchmark PRIVATE ${PCL_LIBRARIES} pthread)
target_compile_options(ikd_tree_benchmark PRIVATE -Wno-unused-variable -Wno-sign-compare)
