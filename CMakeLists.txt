cmake_minimum_required(VERSION 3.0.2)
project(a1_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a1_driver
#  CATKIN_DEPENDS ropscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

