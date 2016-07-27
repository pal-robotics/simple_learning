cmake_minimum_required(VERSION 2.8.3)
project(simple_learning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Learn.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS  message_runtime
)


#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/simple_learn.py
   scripts/learn_server.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


foreach (dir launch)
  install(DIRECTORY ${dir}/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

