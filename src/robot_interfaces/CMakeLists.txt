cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Distance.msg"
  "msg/Battery.msg"
  "msg/RobotCommand.msg"
  "msg/Safety.msg"
  "msg/ComponentStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()