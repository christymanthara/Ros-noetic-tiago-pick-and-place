cmake_minimum_required(VERSION 3.0.2)
project(rosnavigatePnP)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  tf
  sensor_msgs
  geometry_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  FILES TiagoMove.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib roscpp std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(tiago_client src/tiago_client.cpp)
add_executable(tiago_server src/tiago_server.cpp src/detector.cpp) # Add detector.cpp here

## Specify libraries to link a library or executable target against
target_link_libraries(tiago_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(tiago_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS tiago_client tiago_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
##install(DIRECTORY include/${PROJECT_NAME}/
##  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
##  FILES_MATCHING PATTERN "*.h"
##  PATTERN ".svn" EXCLUDE
##)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosnavigatePnP.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

