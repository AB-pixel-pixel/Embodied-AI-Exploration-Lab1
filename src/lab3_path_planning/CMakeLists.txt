cmake_minimum_required(VERSION 3.0.2)
project(lab3_path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  move_base_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab3_path_planning
#  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs move_base_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/path_planning_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
