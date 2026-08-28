cmake_minimum_required(VERSION 3.16)
project(navigation_contracts LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SampleOdometryAtTime.srv"
  "msg/NavigationGoal.msg"
  "msg/RouteSnapshot.msg"
  "msg/NavigationModeStatus.msg"
  "msg/RegisteredScan.msg"
  "msg/PropagatedOdometry.msg"
  "msg/EstimatorHealth.msg"
  "msg/NavigationCommand.msg"
  DEPENDENCIES builtin_interfaces nav_msgs std_msgs geometry_msgs sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(navigation_common rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_typed_interfaces test/test_typed_interfaces.cpp)
  target_include_directories(test_typed_interfaces PRIVATE include)
  ament_target_dependencies(test_typed_interfaces
    builtin_interfaces geometry_msgs nav_msgs navigation_common sensor_msgs std_msgs)
  rosidl_get_typesupport_target(navigation_contracts_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_typed_interfaces
    "${navigation_contracts_typesupport_target}")
endif()

ament_package()
