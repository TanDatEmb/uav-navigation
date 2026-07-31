cmake_minimum_required(VERSION 3.22)
project(uav_simulation)

find_package(ament_cmake REQUIRED)

foreach(asset_dir
    bridge
    launch
    models
    rviz
    worlds
)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${asset_dir}")
    install(
      DIRECTORY "${asset_dir}"
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_m1_harness_assets.py")
    ament_add_pytest_test(
      test_m1_harness_assets
      test/test_m1_harness_assets.py
    )
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_px4_mid360_assets.py")
    ament_add_pytest_test(
      test_px4_mid360_assets
      test/test_px4_mid360_assets.py
    )
  endif()
endif()

ament_package()
