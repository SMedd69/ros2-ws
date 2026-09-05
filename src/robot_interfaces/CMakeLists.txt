cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB MSG_FILES CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  "msg/*.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()