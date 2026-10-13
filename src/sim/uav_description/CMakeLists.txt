cmake_minimum_required(VERSION 3.22)
project(uav_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_sensor_frames_contract
    test/test_sensor_frames_contract.py
  )
endif()

ament_package()
