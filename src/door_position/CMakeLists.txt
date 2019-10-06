cmake_minimum_required(VERSION 2.8.3)
project(door_position)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    message_runtime
    roslaunch
    door_detector
)
# roslaunch_add_file_check(launch)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Door.msg
    DoorList.msg
    DoorPose.msg
)

catkin_python_setup()

## Generate added messages and services with any dependencies
generate_messages(
    DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
    message_generation
    message_runtime
    rospy std_msgs
    sensor_msgs
    geometry_msgs
    door_detector
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

catkin_install_python(PROGRAMS bin/door_position
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
