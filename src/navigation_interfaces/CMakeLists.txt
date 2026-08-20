cmake_minimum_required(VERSION 3.16)
project(navigation_interfaces LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SampleOdometryAtTime.srv"
  "msg/NavigationGoal.msg"
  "msg/NavigationModeStatus.msg"
  "msg/LidarMappingObservation.msg"
  "msg/PlannedTrajectory.msg"
  "msg/TrajectoryCandidate.msg"
  "msg/PlannedTrajectoryBundle.msg"
  "msg/TrajectorySegment.msg"
  "msg/TrajectoryBundle.msg"
  "msg/SafetyActivation.msg"
  "msg/PlannerCycleTrace.msg"
  DEPENDENCIES builtin_interfaces nav_msgs std_msgs geometry_msgs sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
