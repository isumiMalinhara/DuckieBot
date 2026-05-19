cmake_minimum_required(VERSION 3.0.2)
project(closed_loop_square)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/closed_loop_square.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
