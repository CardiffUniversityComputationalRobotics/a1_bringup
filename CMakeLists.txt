cmake_minimum_required(VERSION 3.0.2)
project(a1_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  unitree_legged_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES a1sdk
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-O3")

## sdk directories
include_directories(/home/$ENV{USER}/ros/noetic/system/src/unitree_legged_sdk/include)
    link_directories(/home/$ENV{USER}/ros/noetic/system/src/unitree_legged_sdk/lib)
string(CONCAT LEGGED_SDK_NAME /home/$ENV{USER}/ros/noetic/system/src/a1_driver/lib/libunitree_legged_sdk_amd64.so)
set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

add_executable(a1_driver src/a1_driver.cpp)
target_link_libraries(a1_driver ${EXTRA_LIBS} ${catkin_LIBRARIES})