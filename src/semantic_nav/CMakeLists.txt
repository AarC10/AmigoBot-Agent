cmake_minimum_required(VERSION 3.0.2)
project(semantic_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES
    GoToTarget.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    sensor_msgs
    actionlib_msgs
    message_runtime
)

catkin_install_python(
  PROGRAMS
    scripts/semantic_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
