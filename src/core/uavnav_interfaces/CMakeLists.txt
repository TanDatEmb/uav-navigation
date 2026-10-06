cmake_minimum_required(VERSION 3.22)
project(uavnav_interfaces LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LioState.msg
  msg/LioHealth.msg
  msg/LioOdometry.msg
  msg/Alignment.msg
  msg/NavCommand.msg
  msg/EventRecord.msg
  msg/Waypoint.msg
  msg/MissionDefinition.msg
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
)

ament_package()
