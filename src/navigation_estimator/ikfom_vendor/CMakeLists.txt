cmake_minimum_required(VERSION 3.22)
project(ikfom_vendor)

find_package(ament_cmake REQUIRED)

# Deliberately an interface target. Upstream IKFoM is pinned in UPSTREAM.md,
# but no upstream implementation is copied or downloaded during a build.
add_library(ikfom_vendor INTERFACE)
add_library(ikfom_vendor::ikfom_vendor ALIAS ikfom_vendor)
target_compile_features(ikfom_vendor INTERFACE cxx_std_20)

install(TARGETS ikfom_vendor EXPORT export_ikfom_vendor)
install(DIRECTORY cmake vendor LICENSES
  DESTINATION share/${PROJECT_NAME}
)
install(FILES UPSTREAM.md PATCHES.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_ikfom_vendor)
ament_package()
