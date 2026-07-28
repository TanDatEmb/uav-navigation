cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Interface-only packaging of the exact upstream ROS message boundary. This
# target intentionally does not build or claim to provide the Livox device
# driver executable or Livox SDK2 transport.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs)

install(FILES LICENSE UPSTREAM.md PATCHES.md
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
