cmake_minimum_required(VERSION 2.8.3)
project(tum_ics_lacquey_gripper_driver)

# Set the rpath (to build deb package)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tum_ics_lacquey_gripper_msgs
    tum_ics_ur_robot_msgs
)

##############################################################################
# TCommon
##############################################################################
find_package(TCommon REQUIRED)
add_definitions(${TCOMMON_DEFINITIONS})
include_directories(${TCOMMON_INCLUDE_DIRS})

##############################################################################
# Qt Library
##############################################################################
message("${BGREEN}+++${NOCOLOR} Compile with ${BYELLOW}Qt5${NOCOLOR}")
find_package(Qt5 COMPONENTS Core Network Widgets REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tum_ics_lacquey_gripper_msgs tum_ics_ur_robot_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

# needed for Qt generated files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

###########
## Build ##
###########

#file(GLOB_RECURSE HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
#	include/*.h)

##############################################################################
# Sections Library Enable for Development
##############################################################################

# add executables
add_library(tum_ics_lacquey_gripper_driver
    src/JointState.cpp
    src/JointStatePub.cpp
    src/GripperDriver.cpp
)

# link target against libs
target_link_libraries(tum_ics_lacquey_gripper_driver
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${TCOMMON_LIBRARIES}
    ${QT_LIBRARIES}
)

target_link_libraries(tum_ics_lacquey_gripper_driver Qt5::Core Qt5::Network Qt5::Widgets)

add_dependencies(tum_ics_lacquey_gripper_driver
    ${tum_ics_lacquey_gripper_msgs_EXPORTED_TARGETS}
    ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
)

##############################################################################
# Applications
##############################################################################
QT5_WRAP_CPP(HEADERS_MOC ${QT_HEADERS})

set(APP_TARGETS
    lacquey_gripper_driver
)

foreach(target ${APP_TARGETS})
        add_executable(${target} src/Applications/main_${target}.cpp ${SOURCES} ${HEADERS_MOC})
        add_dependencies(${target}
            ${tum_ics_lacquey_gripper_msgs_EXPORTED_TARGETS}
            ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
        )

        target_link_libraries(${target}
            tum_ics_lacquey_gripper_driver
            ${catkin_LIBRARIES}
            ${roscpp_LIBRARIES}
            ${TCOMMON_LIBRARIES}
            ${QT_LIBRARIES}
        )
        
        target_link_libraries(${target} Qt5::Core)

endforeach()

#############
## Install ##
#############

######################################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the debtools
# This function is controlled with the global variable -DDEB_BUILD (true/false)
include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
ics_ros_debgen()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS tum_ics_lacquey_gripper_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#foreach(dir launch meshes ...)
#    install(DIRECTORY ${dir}/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)
