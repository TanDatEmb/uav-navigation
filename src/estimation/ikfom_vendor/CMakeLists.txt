cmake_minimum_required(VERSION 3.22)
project(ikfom_vendor)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(IKFOM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/IKFoM")
set(IKFOM_TOOLKIT "${IKFOM_ROOT}/IKFoM_toolkit")
if(NOT EXISTS "${IKFOM_TOOLKIT}/esekfom/esekfom.hpp" OR
   NOT EXISTS "${IKFOM_TOOLKIT}/mtk/build_manifold.hpp")
  message(FATAL_ERROR
    "Pinned IKFoM source is missing. Restore vendor/IKFoM at the commit "
    "recorded in UPSTREAM.md; network downloads during build are forbidden.")
endif()

add_library(ikfom_vendor INTERFACE)
add_library(ikfom_vendor::ikfom_vendor ALIAS ikfom_vendor)
target_compile_features(ikfom_vendor INTERFACE cxx_std_20)
target_include_directories(ikfom_vendor INTERFACE
  "$<BUILD_INTERFACE:${IKFOM_TOOLKIT}>"
  "$<INSTALL_INTERFACE:include/ikfom_vendor/IKFoM_toolkit>")
target_link_libraries(ikfom_vendor INTERFACE Eigen3::Eigen)

install(TARGETS ikfom_vendor EXPORT export_ikfom_vendor)
install(DIRECTORY "${IKFOM_TOOLKIT}/"
  DESTINATION include/ikfom_vendor/IKFoM_toolkit)
install(DIRECTORY cmake LICENSES DESTINATION share/${PROJECT_NAME})
install(FILES "${IKFOM_ROOT}/LICENSE" "${IKFOM_ROOT}/README.md"
  DESTINATION share/${PROJECT_NAME}/upstream)
install(FILES UPSTREAM.md PATCHES.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ikfom_upstream_smoke
    test/test_ikfom_upstream_smoke.cpp)
  target_link_libraries(test_ikfom_upstream_smoke ikfom_vendor)
endif()

ament_export_targets(export_ikfom_vendor)
ament_export_dependencies(Eigen3)
ament_package()
