cmake_minimum_required(VERSION 3.0.2)
project(pickit_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/vision_pickit.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
