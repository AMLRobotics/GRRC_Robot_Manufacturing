cmake_minimum_required(VERSION 3.0.2)
project(stl_reader)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
  tf
)

catkin_package(
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/stl_tool_path.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
