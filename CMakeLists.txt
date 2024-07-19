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
  tiago_iaslab_simulation
  roslaunch
  cv_bridge
  image_transport
  control_msgs
  apriltag_ros
  moveit_ros_planning_interface
  tf
  tf_conversions
)

find_package(OpenCV REQUIRED COMPONENTS
opencv_highgui)


roslaunch_add_file_check(launch)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
   FILES 
   Detection.srv
   Scan.srv
)


add_action_files(
  FILES TiagoMove.action
  ArmMove.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  std_msgs
  geometry_msgs
  apriltag_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib roscpp std_msgs message_runtime geometry_msgs tiago_iaslab_simulation
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
  ${catkin_INCLUDE_DIRS}/rosnavigatePnP
)

# List of source files for navigation_node
set(SOURCE_FILES_NAVIGATION
  src/tiago_server.cpp
  # src/utils/navigation_methods.cpp
  src/utils/utils.cpp  
)
# List of source files for apriltag_detection_node
set(SOURCE_FILES_DETECTION
  src/apriltagdetected.cpp 
  src/utils/utils.cpp 
)

# List of source files for client_node
set(SOURCE_FILES_CLIENT
  src/tiago_client.cpp
  # src/utils/navigation_methods.cpp
  src/utils/utils.cpp  
)

#List of source file for scan_node
set(SOURCE_FILES_SCAN
  src/detector.cpp 
  src/utils/utils.cpp 
  src/utils/scan_methods.cpp
)

#List of source file for manipulation_node
set(SOURCE_FILES_MANIPULATION
  src/arm_manipulator.cpp 
  src/utils/utils.cpp 
)

# ## Declare a C++ executable
# add_executable(tiago_client src/tiago_client.cpp)
# add_executable(tiago_server src/tiago_server.cpp src/detector.cpp) # Add detector.cpp here

# ## Specify libraries to link a library or executable target against
# target_link_libraries(tiago_client
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# target_link_libraries(tiago_server
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# ## Ensure that messages are generated before your executables are built
# add_dependencies(tiago_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(tiago_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# naigation_node executable
add_executable(navigation_node ${SOURCE_FILES_NAVIGATION})
target_link_libraries(navigation_node ${catkin_LIBRARIES})
add_dependencies(navigation_node ${rosnavigatePnP_EXPORTED_TARGETS})

#detection_node executable
add_executable(detection_node ${SOURCE_FILES_DETECTION})
target_link_libraries(detection_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(detection_node ${rosnavigatePnP_EXPORTED_TARGETS})

#manipulation_node executable
add_executable(manipulation_node ${SOURCE_FILES_MANIPULATION})
target_link_libraries(manipulation_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(manipulation_node ${rosnavigatePnP_EXPORTED_TARGETS})

#scan_node executable
add_executable(scan_node ${SOURCE_FILES_SCAN} )
target_link_libraries(scan_node ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
add_dependencies(scan_node ${rosnavigatePnP_EXPORTED_TARGETS})

# client_node executable
add_executable(client_node ${SOURCE_FILES_CLIENT})
target_link_libraries(client_node ${catkin_LIBRARIES})
add_dependencies(client_node ${rosnavigatePnP_EXPORTED_TARGETS})


#############
## Install ##
#############

# ## Mark executables for installation
# install(TARGETS tiago_client tiago_server
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
