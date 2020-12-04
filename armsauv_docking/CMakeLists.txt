cmake_minimum_required(VERSION 2.8.3)
project(armsauv_docking)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  armsauv_control
  armsauv_description
  roscpp
  rospy
  tf
  #  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
  armsauv_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

# add_message_files(
#   FILES
#   Usbl.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  roscpp
  std_msgs  # Or other packages containing msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES armsauv_docking
   CATKIN_DEPENDS armsauv_control armsauv_description roscpp rospy std_msgs nav_msgs geometry_msgs armsauv_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(common
  src/common.cpp
  src/file_writer.cpp
)

add_library(usbl_simulator
  src/usbl_simulator.cpp
)

add_library(docking_recorder
  src/docking_recorder.cpp
)

add_dependencies(usbl_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(docking_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(usbl_simulator_node src/usbl_simulator_node.cpp)

add_executable(docking_recorder_node src/docking_recorder_node.cpp)

add_dependencies(usbl_simulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(docking_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(usbl_simulator_node 
  usbl_simulator
  common
  ${catkin_LIBRARIES}
)

target_link_libraries(docking_recorder_node
  docking_recorder
  common
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  scripts/arm_odom_generator.py
  scripts/auv_odom_generator.py
  scripts/docking_demo.py
  scripts/usbl_generator.py
  scripts/usbl_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

