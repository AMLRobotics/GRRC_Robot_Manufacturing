#!/usr/bin/env python3
import rospy
import threading
import time
from pynput import keyboard
import actionlib
from ur_dashboard_msgs.msg import SetModeAction, \
                                  SetModeGoal, \
                                  RobotMode
from ur_dashboard_msgs.srv import GetRobotMode, \
                                  GetProgramState, \
                                  GetLoadedProgram, \
                                  GetSafetyMode, \
                                  Load
from controller_manager_msgs.srv import SwitchControllerRequest, \
                                        SwitchController
from std_srvs.srv import Trigger
import std_msgs.msg
from std_msgs.msg import Bool
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates
from ur_ros_cartesian_control.msg import TargetJointDegs
from ur_ros_cartesian_control.srv import *
import numpy as np

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "ur_shoulder_pan_joint",
    "ur_shoulder_lift_joint",
    "ur_elbow_joint",
    "ur_wrist_1_joint",
    "ur_wrist_2_joint",
    "ur_wrist_3_joint",
]

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
        print("UR Class is Initializing")

        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cnt = 0
        timeout = rospy.Duration(30)
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
        self.trajectory_client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        if not self.trajectory_client.wait_for_server(timeout):
            print(
                "Could not reach cartesian passthrough controller action. Make sure that the driver is actually running."
                " Msg: {}".format(err))

        self.script_publisher = rospy.Publisher("/ur_hardware_interface/script_command", std_msgs.msg.String, queue_size=1)

        # Running up the Manipulator
        resp = self.s_connectToDashboardServer()
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)
        self.set_robot_to_mode(RobotMode.RUNNING)
        rospy.sleep(10)
        self.s_loadProgram("/programs/ros.urp")
        rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
        resp = self.s_playProgram() 
        rospy.sleep(0.5)
        self.switch_on_controller("pos_joint_traj_controllerr")
        rospy.sleep(0.5)
        self.switch_on_controller("scaled_pos_joint_traj_controller")
        rospy.sleep(0.5)
        rospy.Subscriber('/scaled_pos_joint_traj_controller/state',JointTrajectoryControllerState, self.callback)
        rospy.Subscriber('/ur_joint_deg_command',TargetJointDegs, self.command_callback)


        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    def command_callback(self, data):
        commands = np.array(data.jointDegs)
        command_lists = np.empty((0, 6), dtype =np.float32)

        for i in range(int(len(commands) / 6)):
            command_lists = np.append(command_lists, np.array([commands[i:i+6]]), axis = 0)

        self.command_move(command_lists)

    def manual_move(self, key):
        # get keyboard input and add number to specific value 
        if key == keyboard.KeyCode(char='c'):
            self.move_flag = True
            # pose list
            pose_list = [[0.2, 0.3, 0.4, 0.0, 90.0, 270.0], 
                        [0.0, 0.3, 0.4, 0.0, 90.0, 270.0]]

            rospy.wait_for_service('/IK_solve')
            joint_list = np.empty((0, 6), dtype = np.float32)
            time_from_start = 2.0
            q = [-np.pi/2.0, -np.pi/2.0, -np.pi/2.0, -np.pi/2.0, np.pi/2.0, -np.pi]
            
            #try:
            for i in pose_list:
                solve_ik = rospy.ServiceProxy('/IK_solve', cartesian2joint)
                print(i)
                joints = solve_ik(i[0], i[1], i[2], i[3], i[4], i[5], q)
                joints = np.array([joints.jointDegs])
                q = joints[0]
                joint_list = np.append(joint_list, joints, axis = 0)

            #except rospy.ServiceException as e:
            #    print("Service call failed: %s"%e)

            print(joint_list)
            
            # Call Cartesian Tcrajectory Function with Changed Pose Parameters
            self.joint_traj(joint_list, time_from_start)
            self.move_flag = False

    def command_move(self, joint_degs):
        self.move_flag = True
        time_from_start = 4.0
        self.joint_traj(joint_degs, time_from_start)
        self.move_flag = False

    # Function for Pressed Keyboard Event
    def on_press(self, key):
        #print('Key %s pressed' % key)
        # check robot is moving, if robot is moving, then do nothing.
        if self.move_flag == False:
            threading.Thread(target=self.manual_move, args=(key,)).start()

    # <- note extra ','
    def on_release(self, key):
        #print('Key %s released' %key)
        if key == keyboard.Key.esc:
            self.finalize()
        return False

    def set_robot_to_mode(self, target_mode):
        goal = SetModeGoal()
        goal.target_robot_mode.mode = target_mode
        goal.play_program = True # we use headless mode during tests

        # This might be a bug to hunt down. We have to reset the program before calling `resend_robot_program`
        goal.stop_program = False
        self.set_mode_client.send_goal(goal)
        self.set_mode_client.wait_for_result()

        return self.set_mode_client.get_result().success

    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""
        srv = SwitchControllerRequest()
        srv.stop_controllers = ALL_CONTROLLERS
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        print(result)

    def joint_traj(self, joint_lists, time_from_start):
        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        print(len(joint_lists))

        for j in range(len(joint_lists)):
            #print(j.shape)
            point = JointTrajectoryPoint()
            point.positions = joint_lists[j].tolist()

            point.time_from_start = rospy.Duration(time_from_start * (j + 1))
            goal.trajectory.points.append(point)

        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()
        #rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def callback(self, data):
        self.position = list(data.actual.positions)
        
    def finalize(self):
        self.set_robot_to_mode(RobotMode.POWER_OFF)
        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('ur_ros_joint_control_node')
    try:
            ur = UR()
    except rospy.ROSInterruptException: pass