cmake_minimum_required(VERSION 3.8)

project(robot_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

include_directories(
  include
)

# ============================================================
# Motor Node
# ============================================================

file(GLOB_RECURSE MOTOR_NODE_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/motor_node/*.cpp"
)

add_executable(motor_node
  ${MOTOR_NODE_SOURCES}
)

ament_target_dependencies(motor_node
  rclcpp
  geometry_msgs
  robot_interfaces
)

# ============================================================
# Lidar Processor Node
# ============================================================

file(GLOB_RECURSE LIDAR_PROCESSOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_processor_node/*.cpp"
)

add_executable(lidar_processor_node
  ${LIDAR_PROCESSOR_SOURCES}
)

ament_target_dependencies(lidar_processor_node
  rclcpp
  sensor_msgs
  robot_interfaces
)

# ============================================================
# Installation
# ============================================================

install(
  TARGETS
    motor_node
    lidar_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

ament_package()