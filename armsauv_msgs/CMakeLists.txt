cmake_minimum_required(VERSION 2.8.3)
project(armsauv_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Usbl.msg
  AuvCtrlInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES armsauv_msgs
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

