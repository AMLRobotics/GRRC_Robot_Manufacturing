cmake_minimum_required(VERSION 3.0.2)
project(can_sending)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  serial
  message_generation
  rosconsole
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  nav_msgs
  rospy
)


################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Pose.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES can_sending
  CATKIN_DEPENDS serial rosconsole roscpp sensor_msgs std_msgs geometry_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include  ${catkin_INCLUDE_DIRS})

add_executable(sending_CAN src/sending_CAN.cpp)

target_include_directories(sending_CAN PRIVATE ${catkin_INCLUDE_DIRS})


target_link_libraries(sending_CAN ${catkin_LIBRARIES})
target_link_libraries(sending_CAN canlib)


add_dependencies(sending_CAN ${catkin_EXPORTED_TARGETS})

install(TARGETS sending_CAN 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


