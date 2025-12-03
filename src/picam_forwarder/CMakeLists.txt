cmake_minimum_required(VERSION 3.0.2)
project(picam_forwarder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/picam_forwarder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)