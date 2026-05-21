cmake_minimum_required(VERSION 3.0.2)
project(ur_ros_cartesian_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  ur_kinematics
  ur_description
  message_generation
  xacro
  robot_state_publisher
  joint_state_publisher
  tf2
)

add_message_files(
  FILES
  TargetJointDegs.msg
)

add_service_files(
  FILES
  joint2cartesian.srv
  cartesian2joint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package( 
  CATKIN_DEPENDS ur_kinematics
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/ur_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/ur_joy_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/ur_point_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS src/ur_joint_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(ur10_ik src/ur10_ik.cpp)

target_link_libraries(ur10_ik ${catkin_LIBRARIES})
set_target_properties(ur10_ik PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")
target_link_libraries(ur10_ik
  ${catkin_LIBRARIES}
  ur5_kin)

add_dependencies(ur10_ik ${catkin_EXPORTED_TARGETS})

install(TARGETS ur10_ik
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)