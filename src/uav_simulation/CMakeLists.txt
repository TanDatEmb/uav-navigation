cmake_minimum_required(VERSION 3.22)
project(uav_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY bridge launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_m1_harness_assets test/test_m1_harness_assets.py)
endif()

ament_package()
