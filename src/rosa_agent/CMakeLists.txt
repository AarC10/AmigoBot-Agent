cmake_minimum_required(VERSION 3.0.2)
project(amigo_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib
  semantic_nav
  vlm_clip
)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/agent_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
