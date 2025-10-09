cmake_minimum_required(VERSION 3.5)
project(hydra_sandbox)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cartesian_planning_server REQUIRED)

install(PROGRAMS
  scripts/gcode_execution.py
  scripts/visualize_collision_geometry.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
