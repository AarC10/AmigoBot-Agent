cmake_minimum_required(VERSION 3.0.2)
project(vlm_clip)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs message_runtime
)

catkin_install_python(
  PROGRAMS
    scripts/clip_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
