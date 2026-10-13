cmake_minimum_required(VERSION 3.22)
project(uavnav_bringup NONE)

# Launch files only (D33/O19); the processes they start live in their own packages.
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_s1_estimation_launch test/test_s1_estimation_launch.py TIMEOUT 60)
endif()

ament_package()
