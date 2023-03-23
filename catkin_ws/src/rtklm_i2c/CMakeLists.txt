cmake_minimum_required(VERSION 3.0.2)
project(rtklm_i2c)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rtklm_i2c
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/rtklm_i2c_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
