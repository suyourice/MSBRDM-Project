cmake_minimum_required(VERSION 2.8.0)
project(tum_ics_lacquey_gripper_msgs)

# Set the rpath (to build deb package)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
    DIRECTORY msg
    FILES
	  GripperState.msg
)

add_service_files(
    DIRECTORY srv
    FILES
	  setGripperState.srv
	  getGripperState.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

catkin_package(
 #INCLUDE_DIRS include
 CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs message_runtime 
)

#############
## Install ##
#############

######################################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the debtools
# This function is controlled with the global variable -DDEB_BUILD (true/false)
include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
ics_ros_debgen()

foreach(dir msg srv)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)


