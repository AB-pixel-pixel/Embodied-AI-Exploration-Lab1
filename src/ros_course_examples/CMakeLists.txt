cmake_minimum_required(VERSION 3.0.2)
project(ros_course_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf
  nav_msgs
  gazebo_ros
  message_generation
)

# add_message_files(
#  FILES
  # Add custom messages here if needed
#)

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_course_examples
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs cv_bridge image_transport tf nav_msgs gazebo_ros message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/topic_publisher.py
  nodes/topic_subscriber.py
  nodes/image_publisher.py
  nodes/move_turtlebot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
