cmake_minimum_required(VERSION 3.0.2)
project(semantic_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  vlm_clip
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/semantic_nav_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
