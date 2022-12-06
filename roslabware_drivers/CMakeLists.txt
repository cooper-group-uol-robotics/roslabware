cmake_minimum_required(VERSION 3.0.2)
project(roslabware_drivers)


find_package(catkin REQUIRED COMPONENTS
  rospy
  roslabware_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roslabware_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB PROGRAM_FILES nodes/*)
catkin_install_python(PROGRAMS ${PROGRAM_FILES}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
