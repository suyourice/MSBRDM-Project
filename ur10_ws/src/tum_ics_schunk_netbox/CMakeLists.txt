cmake_minimum_required(VERSION 3.0.2)
project(tum_ics_schunk_netbox)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  std_srvs
)

find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
#add_service_files( FILES rft_operation.srv)
#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES tum_ics_schunk_netbox
  CATKIN_DEPENDS geometry_msgs message_generation roscpp std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp executable
add_executable(minimal_node
	src/minimal_node_main.cpp
)
add_dependencies(minimal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(minimal_node ${catkin_LIBRARIES})
