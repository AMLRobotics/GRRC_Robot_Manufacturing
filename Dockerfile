# syntax=docker/dockerfile:1
FROM osrf/ros:noetic-desktop-full

# Install Packages for Installing Python Packages
RUN /bin/bash -c "apt-get update -y &&\
    apt-get install -y python3-pip &&\
    apt-get install -y python3-all-dev &&\
    apt-get install -y python3-rospkg &&\
    apt-get install -y ros-noetic-desktop-full --fix-missing"

RUN /bin/bash -c "apt-get update -y &&\
    apt-get install -y build-essential &&\
    apt-get install -y libssl-dev &&\
    apt-get install -y libffi-dev &&\
    apt-get install -y libxml2-dev &&\
    apt-get install -y libxslt1-dev &&\
    apt-get install -y git &&\
    apt-get install -y zlib1g-dev"

# Create catkin workspace
# Install UR Robot Driver
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/noetic/setup.bash &&\
    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws &&\
    git clone https://github.com/Hangijun/Universal_Robots_ROS_Driver_UR5.git src/Universal_Robots_ROS_Driver &&\
    git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot &&\    
    apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    cd ~/catkin_ws &&\
    catkin_make &&\
    chmod 777 src/Universal_Robots_ROS_Driver/ur_robot_driver/launch/ur5_cartesian_passthrough_bringup.launch &&\
    source devel/setup.bash"

# Install Pynput Package for Keyboard Input Control
RUN python3 -m pip install pynput

# Install Cartesian Controller Packages
RUN /bin/bash -c "apt-get update &&\
    source /opt/ros/noetic/setup.bash &&\
    cd ~/catkin_ws &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_cartesian_control_msgs.git src/Universal_Robots_ROS_cartesian_control_msgs &&\
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian.git src/Universal_Robots_ROS_controllers_cartesian &&\
    sudo apt update -qq &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    catkin_make &&\
    source devel/setup.bash"

# Install scipy
RUN python3 -m pip install scipy

# Install Custom Package(ur_ros_joint_control)
RUN /bin/bash -c "cd ~/catkin_ws &&\
    git clone https://github.com/Hangijun/ur_ros_joint_control.git src/ur_ros_joint_control &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"

# Additional
RUN /bin/bash -c "apt-get update -y &&\
    apt-get install -y python3-tk &&\
    apt-get install -y tk-dev"

# Install Custom Package(ur_ros_cartesian_control)
RUN /bin/bash -c "cd ~/catkin_ws &&\
    git clone https://github.com/Hangijun/ur_ros_cartesian_control.git src/ur_ros_cartesian_control &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make &&\
    source devel/setup.bash"