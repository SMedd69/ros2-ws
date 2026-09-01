cmake_minimum_required(VERSION 3.8)
project(robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

include_directories(
  include
)

add_executable(motor_node
  src/main.cpp
  src/motor_node.cpp
  src/motor_controller.cpp
  src/motor.cpp
)

ament_target_dependencies(
  motor_node
  rclcpp
  robot_interfaces
)

install(
  TARGETS
  motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()