cmake_minimum_required(VERSION 3.0.2)
project(nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  message_generation
  roslaunch
  roscpp
  robot_upstart
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

catkin_python_setup()

add_message_files(
  FILES
  Command.msg
  Dubins.msg
 )

 generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav
  CATKIN_DEPENDS geometry_msgs mavros message_generation message_runtime roscpp rospy controller_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/nav/nodes/trajectory_publisher.py
  src/nav/nodes/commander_node.py
  src/nav/nodes/dummy_planner_node.py
  src/nav/functions.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

