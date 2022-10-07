cmake_minimum_required(VERSION 3.0.2)
project(a1_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## sdk directories
include_directories(include)
link_directories(lib)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a1sdk
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(a1_driver src/a1_driver.cpp)
target_link_libraries(a1_driver ${EXTRA_LIBS} ${catkin_LIBRARIES})

