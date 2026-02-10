#!/usr/bin/env python3

# rospy for use ros with python
import rospy

# thread module of python
import threading

# time module of python (refer to unix time)
import time

# keyboard module of pynput python package for keyboard input
from pynput import keyboard

# actionlib for Connect with ROS Action Server
import actionlib

# ur dashboard messages to use with ros service or ros action
from ur_dashboard_msgs.msg import SetModeAction, \
                                    SetModeGoal, \
                                    RobotMode

# ur dashboard service messages to require service to ur controller
from ur_dashboard_msgs.srv import GetRobotMode, \
                                    GetProgramState, \
                                    GetLoadedProgram, \
                                    GetSafetyMode, \
                                    Load

# controller manager service messages to choose controller for ur robot
from controller_manager_msgs.srv import SwitchControllerRequest, \
                                        SwitchController

# Trigger Module for standard service
from std_srvs.srv import Trigger

# standard messages for various purpose (e.g. String)
import std_msgs.msg

# cartesian control messages to control ur robot in cartesian coordinate
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, \
                                        FollowCartesianTrajectoryGoal, \
                                        FollowCartesianTrajectoryResult, \
                                        CartesianTrajectoryPoint

# TFMessage module of tf2 messages, tf means transformation
# It used to get information of end-effector pose 
from tf2_msgs.msg import TFMessage

# Rotation module of scipy package
# It used for convert quaternion to euler or euler to quaternion
from scipy.spatial.transform import Rotation

from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates

#from ur_ros_cartesian_control.srv import *
#from ur_ros_cartesian_control.msg import * 

ALL_CONTROLLERS = [
        "scaled_pos_joint_traj_controller",
        "pos_joint_traj_controller",
        "scaled_vel_joint_traj_controller",
        "vel_joint_traj_controller",
        "joint_group_vel_controller",
        "forward_joint_traj_controller",
        "forward_cartesian_traj_controller",
        "twist_controller",
        "pose_based_cartesian_traj_controller",
        "joint_based_cartesian_traj_controller",
        ]

class UR():
    def __init__(self):

        # Initialize UR Class
        print("UR Class is Initializing")



        # Class Variables Initialization
        # ========================================
        # timeout for wait any server it connect
        timeout = rospy.Duration(30)
        # quaternion pose variable, we get pose of end-effector to this variable.
        self.pose_quat = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # time for unit control of ur robot
        self.time_from_start=0.01

        # set flag for control ur robot
        self.move_flag = False
        
        # ROS Service Initialization
        # ========================================
        # Get Current Robot Mode
        self.s_getRobotMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_robot_mode', GetRobotMode)
        # Get Current Program State
        self.s_getProgramState = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        # Get Current Loaded Program
        self.s_getLoadedProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_loaded_program', GetLoadedProgram)
        # Get Current Safety Mode State
        self.s_getSafetyMode = rospy.ServiceProxy('/ur_hardware_interface/dashboard/get_safety_mode', GetSafetyMode)
        
        # Connect to Dashboard Server
        self.s_connectToDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        # Quit from Dashboard Server
        self.s_quitFromDashboardServer = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        # Load Program
        self.s_loadProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        # Play Program
        self.s_playProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        # Stop Program
        self.s_stopProgram = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        


        # ROS Action Initialization
        # ========================================
        # Connect to Set Mode Action Server from Client
        self.set_mode_client = actionlib.SimpleActionClient(
            '/ur_hardware_interface/set_mode', SetModeAction)
        try:
            self.set_mode_client.wait_for_server(timeout)
            print("set mode action client is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err))
        # Connect to Switch Controller Action Server from Client
        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(timeout)
            print("controller switch service is on")
        except rospy.exceptions.ROSException as err:
            print(
                "Could not reach controller switch service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
        # Connect to Cartesian Controller Action Server from Client
        self.cartesian_passthrough_trajectory_client = actionlib.SimpleActionClient(
            '/forward_cartesian_traj_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not self.cartesian_passthrough_trajectory_client.wait_for_server(timeout):
            self.fail(
                "Could not reach cartesian passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        # Connect to Gripper -Han
        self.set_io_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        try:
            self.set_io_client.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach set_io service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
        
        self.set_io_client(1, 17, 0)
        self.set_io_client(1, 16, 1)

        # ROS Publisher & Subscriber Initialization
        # ========================================
        # script publisher for specific purpose
        self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)
        # Subscribe TF Information of the End-Effector
        sub = rospy.Subscriber("/tf", TFMessage, self.callback)
        rospy.Subscriber('str_pos', std_msgs.msg.String, self.move_callback)
              
        
        #self.gripper_cmd = RobotiqVacuumGrippers_robot_output()

        # Running up the Manipulator
        # ========================================
        # Connect to Dashboard Server
        resp = self.s_connectToDashboardServer()
        # Power Off the Robot (for the situation the robot is powered on)
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)
        # Set Robot Mode to Running
        # RUNNING : Execute Power ON & Break Release
        self.set_robot_to_mode(RobotMode.RUNNING)
        rospy.sleep(10)
        # Load Program that we made already
        # It works with Real Robot but URSim
        self.s_loadProgram("/programs/ros.urp")
        # Play the Program that we Load
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        resp = self.s_playProgram()
        rospy.sleep(0.5)
        # Switch the Controller for UR Robot to Forward Cartesian Trajectory Controller
        self.switch_on_controller("pos_joint_traj_controllerr")
        rospy.sleep(0.5)
        self.switch_on_controller("forward_cartesian_traj_controller")
        rospy.sleep(0.5)



        # Event handler for Keyboard Input 
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    # Function for Pressed Keyboard Event
    def on_press(self, key):
        #print('Key %s pressed' % key)
        # check robot is moving, if robot is moving, then do nothing.
        if self.move_flag == False:
            threading.Thread(target=self.manual_move, args=(key,)).start()

    # Function for Released Keyboard Event
    def on_release(self, key):
        #print('Key %s released' %key)
        if key == keyboard.Key.esc:
            self.finalize()
            return False

    # Callback Function to get the TF Message
    def callback(self, data):
        #print(data.transforms[0].transform)
        self.pose_quat[0] = data.transforms[0].transform.translation.x
        self.pose_quat[1] = data.transforms[0].transform.translation.y
        self.pose_quat[2] = data.transforms[0].transform.translation.z
        self.pose_quat[3] = data.transforms[0].transform.rotation.x
        self.pose_quat[4] = data.transforms[0].transform.rotation.y
        self.pose_quat[5] = data.transforms[0].transform.rotation.z
        self.pose_quat[6] = data.transforms[0].transform.rotation.w
    
     # Function for UR robot's gripper
    def gripper_control(self, gripper_on):
        if gripper_on:
            # For Two finger gripper
            # self.set_io_client(1, 16, 1)
            # Vaccum
            self.set_io_client(1, 16, 0)
            self.set_io_client(1, 17, 1)
        else:
            # For Two finger gripper
            # self.set_io_client(1, 16, 0)
            # Vaccum
            self.set_io_client(1, 17, 0)
            self.set_io_client(1, 16, 1)

    # Callback Function to get the pose data
    def move_callback(self, pos_data):
        move_dir = pos_data.data.split(' ')
        move_pos = []
        grip_dir = []

        for i in range(len(move_dir)):
            move_dir[i] = move_dir[i].split(',')
            move_pos.append(move_dir[i][:6])
            grip_dir.append(move_dir[i][6])

        if  not isinstance(move_pos[0], list):
            move_pos = [move_pos]

        #uvprint(move_pos)
        for i in range(len(move_pos)):
            for j in range(len(move_pos[0])):
                move_pos[i][j] = float(move_pos[i][j])
        
        if self.move_flag == False:
            self.move(move_pos)
        
    #Function of autonomous movement control
    # Convert Quaternion to Euler
    # Change value of euler pose and call the control function(cartesian_traj)
    def move(self, pose):
        self.move_flag = True        
        time_from_start = 3.0

        #for l in pose:
        # check cartesian limit to 1.0 m of the 3d circle
        for l in pose:
            check = self.check_cartesian_limit(l[0], l[1], l[2], l[3], l[4], l[5])
            if check == False: return

        # Call Cartesian Trajectory Function with Changed Pose Parameters
        self.cartesian_traj(pose, time_from_start)
        self.move_flag = False

    # Function for take care of Keyboard Input
    # Convert Quaternion to Euler
    # Change value of euler pose and call the control function(cartesian_traj)
    def manual_move(self, key):

        # get keyboard input and add number to specific value 
        if key == keyboard.KeyCode(char='c'):
            self.move_flag = True
            # pose list
            pose_list = [[0.0, -0.3, 0.4, 180.0, 0.0, 0.0], [0.0, -0.3, 0.4, 270.0, 0.0, 0.0], [0.0, -0.3, 0.4, 270.0, 0.0, 90.0], [0.0, -0.3, 0.4, 180.0, 0.0, 0.0]]

            time_from_start = 8.0

            for l in pose_list:
                # check cartesian limit to 1.0 m of the 3d circle
                check = self.check_cartesian_limit(l[0], 
                                            l[1], 
                                            l[2],
                                            l[3],
                                            l[4],
                                            l[5])
                if check == False: return

            # Call Cartesian Trajectory Function with Changed Pose Parameters
            self.cartesian_traj(pose_list, time_from_start)
            self.move_flag = False
            
            #rospy.sleep(0.1)

        if key == keyboard.KeyCode(char='v'):
            self.move_flag = True
            for i in range(500):
                # pose list
                pose_list = [[0.0, -0.4, 0.4, 180.0, 0.0, 0.0], [0.0, -0.3, 0.4, 180.0, 90.0, 90.0], [0.0, -0.3, 0.4, 180.0, 0.0, 0.0]]

                time_from_start = 4.0

                for l in pose_list:
                    # check cartesian limit to 1.0 m of the 3d circle
                    check = self.check_cartesian_limit(l[0], 
                                                l[1], 
                                                l[2],
                                                l[3],
                                                l[4],
                                                l[5])
                    if check == False: return

                # Call Cartesian Trajectory Function with Changed Pose Parameters
                self.cartesian_traj(pose_list, time_from_start)
                rospy.sleep(1.0)
            
                # pose list
                pose_list = [[-0.139, -0.390, 0.4, 180.0, 0.0, 0.0],
                            [-0.139, -0.390, 0.162, 180.0, 0.0, 0.0]]

                time_from_start = 3.0

                for l in pose_list:
                    # check cartesian limit to 1.0 m of the 3d circle
                    check = self.check_cartesian_limit(l[0], 
                                                l[1], 
                                                l[2],
                                                l[3],
                                                l[4],
                                                l[5])
                    if check == False: return

                # Call Cartesian Trajectory Function with Changed Pose Parameters
                for i in range(len(pose_list)):
                    pose = [pose_list[i]]
                    self.cartesian_traj(pose, time_from_start)
                rospy.sleep(1.0)

                # pose list
                pose_list = [[0.137, -0.390, 0.162, 180.0, 0.0, 0.0],
                            [0.137, -0.380, 0.162, 180.0, 0.0, 0.0], 
                            [-0.139, -0.380, 0.162, 180.0, 0.0, 0.0],
                            [-0.139, -0.370, 0.162, 180.0, 0.0, 0.0], 
                            [0.137, -0.370, 0.162, 180.0, 0.0, 0.0],
                            [0.137, -0.360, 0.162, 180.0, 0.0, 0.0], 
                            [-0.139, -0.360, 0.162, 180.0, 0.0, 0.0],
                            [-0.139, -0.350, 0.162, 180.0, 0.0, 0.0], 
                            [0.137, -0.350, 0.162, 180.0, 0.0, 0.0],
                            [0.137, -0.340, 0.162, 180.0, 0.0, 0.0], 
                            [-0.139, -0.340, 0.162, 180.0, 0.0, 0.0],
                            [-0.139, -0.330, 0.162, 180.0, 0.0, 0.0], 
                            [0.137, -0.330, 0.162, 180.0, 0.0, 0.0],
                            [0.137, -0.320, 0.162, 180.0, 0.0, 0.0], 
                            [-0.139, -0.320, 0.162, 180.0, 0.0, 0.0],
                            [-0.139, -0.310, 0.162, 180.0, 0.0, 0.0], 
                            [0.137, -0.310, 0.162, 180.0, 0.0, 0.0],
                            [0.137, -0.300, 0.162, 180.0, 0.0, 0.0], 
                            [-0.139, -0.300, 0.162, 180.0, 0.0, 0.0],
                            [-0.139, -0.290, 0.162, 180.0, 0.0, 0.0], 
                            [0.137, -0.290, 0.162, 180.0, 0.0, 0.0],
                            [0.137, -0.280, 0.162, 180.0, 0.0, 0.0], 
                            [-0.139, -0.280, 0.162, 180.0, 0.0, 0.0],
                            [-0.139, -0.270, 0.162, 180.0, 0.0, 0.0], 
                            [0.137, -0.270, 0.162, 180.0, 0.0, 0.0],
                            [0.137, -0.260, 0.162, 180.0, 0.0, 0.0], 
                            [-0.139, -0.260, 0.162, 180.0, 0.0, 0.0]]

                time_from_start = 2.0

                for l in pose_list:
                    # check cartesian limit to 1.0 m of the 3d circle
                    check = self.check_cartesian_limit(l[0], 
                                                l[1], 
                                                l[2],
                                                l[3],
                                                l[4],
                                                l[5])
                    if check == False: return

                # Call Cartesian Trajectory Function with Changed Pose Parameters
                for i in range(len(pose_list)):
                    pose = [pose_list[i]]
                    if i % 2 == 0:
                        time_from_start = 2.0
                    else:
                        time_from_start = 0.4

                    self.cartesian_traj(pose, time_from_start)
                self.move_flag = False
                rospy.sleep(1.0)

                # pose list
                pose_list = [[-0.139, -0.325, 0.4, 180.0, 0.0, 0.0],
                            [0.0, -0.4, 0.4, 180.0, 0.0, 0.0]]

                time_from_start = 4.0

                for l in pose_list:
                    # check cartesian limit to 1.0 m of the 3d circle
                    check = self.check_cartesian_limit(l[0], 
                                                l[1], 
                                                l[2],
                                                l[3],
                                                l[4],
                                                l[5])
                    if check == False: return

                # Call Cartesian Trajectory Function with Changed Pose Parameters
                self.cartesian_traj(pose_list, time_from_start)
                rospy.sleep(1.0)

    # Function for Control UR Robot in the Cartesian Coordinate
    def cartesian_traj(self, pose_list, time_from_start=0.01):     
        goal = FollowCartesianTrajectoryGoal()

        for l in pose_list:
            point = CartesianTrajectoryPoint()
            point.pose.position.x = l[0]
            point.pose.position.y = l[1]
            point.pose.position.z = l[2]

            rot = Rotation.from_euler('xyz', [l[3], l[4], l[5]], degrees=True)
            rot_quat = rot.as_quat()
            #print(rot_quat)
            point.acceleration.linear.x = 0.5
            point.acceleration.linear.y = 0.5
            point.acceleration.linear.z = 0.5

            point.acceleration.angular.x = 0.5
            point.acceleration.angular.y = 0.5
            point.acceleration.angular.z = 0.5


            point.pose.orientation.x = -rot_quat[0]
            point.pose.orientation.y = -rot_quat[1]
            point.pose.orientation.z = -rot_quat[2]
            point.pose.orientation.w = -rot_quat[3]

            #print(point.pose)

            point.time_from_start = rospy.Duration(secs = time_from_start)
            goal.trajectory.points.append(point)
        
        goal.goal_time_tolerance = rospy.Duration(0.6)
        
        self.cartesian_passthrough_trajectory_client.send_goal(goal)
        self.cartesian_passthrough_trajectory_client.wait_for_result()
        #print(self.cartesian_trajectory_client.get_result())

        #rospy.loginfo("Received result SUCCESSFUL")

    # if the robot trying to reach point which is not stable,
    # there would be error from the robot controller
    # to prevent that situation, we set the limit for the controller
    def check_cartesian_limit(self, x, y, z, roll, pitch, yaw):
        if (pow(x,2)+pow(y,2)+pow(z,2) > 1.0) \
                or (pow(x,2)+pow(y,2)+pow(z,2) < 0.1):
                #or y > 0.0 or z < 0.0 \
                #or (roll > -135.0 and roll < 135.0) \
                #or (pitch < -45.0 or pitch > 45.0):
                #or (yaw > -135.0 and yaw < 135.0): 
            return False
        else: 
            return True     

    # Function to Set Robot Mode
    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode.mode = target_mode
        goal.play_program = True # we use headless mode during tests
        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = False

        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()
        return self.set_mode_client.get_result()

    # Function to Switch the Controller for UR Robot
    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        print(result)

    # Function to Power OFF of UR Robot When the Program is over
    def finalize(self):
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)

# Main, We name the node's name here and Create UR Instance
if __name__ == '__main__':
    rospy.init_node('ur_ros_cartesian_control_node') 
    try:
        ur = UR()
    except rospy.ROSInterruptException: pass
