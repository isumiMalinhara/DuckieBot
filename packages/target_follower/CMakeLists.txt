cmake_minimum_required(VERSION 2.8.3)
project(target_follower)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  roscpp
  duckietown_msgs
)

catkin_package()

# Mark scripts as executable
catkin_install_python(PROGRAMS
  src/target_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
