cmake_minimum_required(VERSION 3.0.2)
project(hydra_sandbox)

if(NOT CMAKE_BUILD_TYPE) 
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  control_msgs
  actionlib
  cartesian_planning_server
  pyrobopath_ros
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  control_msgs
  actionlib
  cartesian_planning_server
  pyrobopath_ros
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/cartesian_trajectory_execution.py
  scripts/schedule_execution.py
  scripts/gcode_execution.py
  scripts/visualize_collision_geometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
