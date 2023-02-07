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
include_directories($ENV{UNITREE_LEGGED_SDK_PATH}/include)
    link_directories($ENV{UNITREE_LEGGED_SDK_PATH}/lib)
    string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
    set(EXTRA_LIBS ${LEGGED_SDK_NAME} lcm)

# set(EXTRA_LIBS -pthread /home/$ENV{USER}/ros/noetic/system/src/unitree_legged_sdk/lib/libunitree_legged_sdk_amd64.so lcm)



add_executable(a1_driver src/a1_driver.cpp)
target_link_libraries(a1_driver ${EXTRA_LIBS} ${catkin_LIBRARIES})


# add_executable(teleop_walk src/teleop.cpp)
# target_link_libraries(teleop_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
# add_dependencies(teleop_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})