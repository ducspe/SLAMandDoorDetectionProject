cmake_minimum_required(VERSION 2.8.3)
project(reporter)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    nav_msgs
    message_runtime
)
roslaunch_add_file_check(launch)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
    rospy
    sensor_msgs
    nav_msgs
    message_runtime
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS bin/reporter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
