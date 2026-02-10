#!/usr/bin/env python3

import os
from stl import mesh
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from pynput import keyboard
import rospy
import math
import threading
import std_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import tf
from math import pi
from scipy.spatial.transform import Rotation as R

class STLToolPath():
    def __init__(self):
        self.tool_height = float(rospy.get_param('/stl_tool_path/tool_diameter'))
        self.str_pub_pos = rospy.Publisher("str_pos", std_msgs.msg.String, queue_size = 10)

        self.trajectory = np.array([[]])
        self.degrees = np.array([[]])

        self.listener = tf.TransformListener()
        self.robot_path = np.array([[]])
        self.trans_ee_to_flange = None
        self.rot_ee_to_flange = None
        self.is_ready = False

        # TF 데이터가 들어올 때까지 잠시 대기
        rospy.sleep(1.0)
        self._init_transform()

        # 가공할 모델의 STL 파일 불러오기
        path = "/root/catkin_ws/src/stl_reader/"
        os.chdir(path)
        self.object = mesh.Mesh.from_file(rospy.get_param('/stl_tool_path/target_filename'))

        # 모델의 최대높이 측정
        self.z_max = 0.0
        for i in range(len(self.object.points)):
            if self.object.v0[i][2] > self.z_max:
                self.z_max = self.object.v0[i][2]

            if self.object.v1[i][2] > self.z_max:
                self.z_max = self.object.v1[i][2]

            if self.object.v2[i][2] > self.z_max:
                self.z_max = self.object.v2[i][2]

    # Function for Released Keyboard Event
    def on_release(self, key):
        #print('Key %s released' %key)
        if key == keyboard.Key.esc:
            self.finalize()
            return False

     # Function for Pressed Keyboard Event
    def on_press(self, key):
        #print('Key %s pressed' % key)
        # check robot is moving, if robot is moving, then do nothing.
        threading.Thread(target=self.manual_move, args=(key,)).start()
    
    # 모델의 외곽면을 따라 가공 경로 생성
    def path_planning(self, model):
        # 가공 경로 평면의 법선벡터
        N = np.array([0, 0, 1])

        # STL 파일과 높이 당 가공 경로 평면의 교차점 저장
        path = np.empty((0, 3), dtype = np.float32)
        direction = np.empty((0, 3), dtype = np.float32)

        for i in range(int(self.z_max / (self.tool_height / 2))):
            #상단 평면 경로 작성
            if i == int(self.z_max / (self.tool_height / 2)) - 1:
                top_points = np.empty((0, 3), dtype = np.float32)
                
                for i in range(len(self.object.points)):
                    normal = self.object.normals[i] / np.linalg.norm(self.object.normals[i], axis = 0)
                    
                    overlap_check = [False, False, False]

                    #중복되는 점들 삭제
                    if len(top_points) > 0:
                        for j in top_points:
                            if j[0] == self.object.v0[i][0] and j[1] == self.object.v0[i][1]:
                                overlap_check[0] = True
                            if j[0] == self.object.v1[i][0] and j[1] == self.object.v1[i][1]:
                                overlap_check[1] = True
                            if j[0] == self.object.v2[i][0] and j[1] == self.object.v2[i][1]:
                                overlap_check[2] = True

                    #상단 평면의 점 선별
                    if self.object.v2[i][2] == self.z_max and (normal == [0, 0, 1.0]).all() and not overlap_check[2]:
                        top_points = np.append(top_points, np.array([self.object.v2[i]]), axis = 0)

                    if self.object.v1[i][2] == self.z_max and (normal == [0, 0, 1.0]).all() and not overlap_check[1]:
                        top_points = np.append(top_points, np.array([self.object.v1[i]]), axis = 0)

                    if self.object.v0[i][2] == self.z_max and (normal == [0, 0, 1.0]).all() and not overlap_check[0]:
                        top_points = np.append(top_points, np.array([self.object.v0[i]]), axis = 0)

                temp = np.empty((0, 1), dtype = int)

                #가장자리가 아닌 점 삭제
                for j in range(len(top_points)):
                    if top_points[j][0] == top_points[j][1]:
                        temp = np.append(temp, np.array([[j]]), axis = 0)

                top_points = np.delete(top_points, temp, axis = 0)
                center_p = np.array([np.mean(top_points[:, 0]), np.mean(top_points[:, 1]), self.z_max])

                angle = np.arctan2(top_points[:, 1] - center_p[1], top_points[:, 0] - center_p[0])

                #순서대로 정렬
                sorted_indices = np.argsort(angle)
                top_points = top_points[sorted_indices]

                temp = top_points.copy()

                for k in range(len(top_points)):
                    top_points[k] = temp[len(top_points) - 1 - k]

                line_count = int(np.sqrt(np.square(top_points[0][0] - center_p[0]) + np.square(top_points[0][1] - center_p[1])) / (self.tool_height * 0.5))

                top_trajectory = top_points.copy()

                #가공 툴의 직경에 비례하여 궤적 생성
                for k in range(line_count - 1):
                    next_line = np.empty((0, 3), dtype = float)

                    for l in range(len(top_points)):
                        d_x = (top_points[l][0] - center_p[0]) / float(line_count)
                        d_y = (top_points[l][1] - center_p[1]) / float(line_count)
                        next_line = np.append(next_line, [[top_points[l][0] - d_x * (k + 1), top_points[l][1] - d_y * (k + 1), self.z_max]], axis = 0)

                        #print(d_x, d_y)

                        #print(len(next_line), len(top_points))

                    top_trajectory = np.append(top_trajectory, next_line.copy(), axis = 0)

                top_direction = np.repeat(np.array([[0, 0, 1.0]]), len(top_trajectory), axis = 0)

                """print(top_trajectory)

                fig3 = plt.figure()
                ax3 = mplot3d.Axes3D(fig3)
                ax3.plot(top_trajectory[:, 0], top_trajectory[:, 1], zs = top_trajectory[:, 2])
                ax3.scatter(top_trajectory[:, 0], top_trajectory[:, 1], top_trajectory[:, 2])"""
                
                #plt.show()
            
            #측면 경로 작성
            else:
                a = np.array([0, 0, (i + 1) * self.tool_height / 2])
                
                for j in range(0, len(model.points)):
                    dots, degs = self.get_plane_intersection(model, j, N, a, path)
                
                    if dots is not None:
                        path = np.append(path, dots, axis = 0)
                        if j == 0:
                            degs = np.append(degs, degs, axis = 0)
                        direction = np.append(direction, degs, axis = 0)

                for j in range(0, 2):
                    path = np.delete(path, i, axis = 0)
                    direction = np.delete(direction, i, axis = 0)

        rear2top = np.array([[path[len(path) - 1][0], path[len(path) - 1][1], self.z_max], [path[len(path) - 1][0], path[len(path) - 1][1], self.z_max]])

        path = np.append(path, rear2top, axis = 0)
        path = np.append(path, top_trajectory, axis = 0)

        rear2top_dir = np.array([direction[len(direction) - 1], [0, 0, 1.0]])

        direction = np.append(direction, rear2top_dir, axis = 0)
        direction = np.append(direction, top_direction, axis = 0)
        self.direction = direction

        self.trajectory = path
        self.degrees = self.cal_angle_acc(-direction) # 벡터를 오일러 각으로 변환
        print(self.degrees)

        for i in range(len(self.trajectory)):
            if i == 0:
                self.robot_path = self.get_flange_target_array(np.array([self.trajectory[i][0], self.trajectory[i][1], self.trajectory[i][2], self.degrees[i][0], self.degrees[i][1], self.degrees[i][2]]))
            else:
                self.robot_path = np.append(self.robot_path, self.get_flange_target_array(np.array([self.trajectory[i][0], self.trajectory[i][1], self.trajectory[i][2], self.degrees[i][0], self.degrees[i][1], self.degrees[i][2]])), axis = 0)


        self.visualization(model, path, direction)

    # 가공 경로 평면과 교차점이 존재하는지 여부 확인
    def check_intersection(self, p, q, a, N):
        P_p = np.dot(N, p) - np.dot(N, a)
        P_q = np.dot(N, q) - np.dot(N, a)
        result = P_p * P_q

        if result > 0:
            return False

        else:
            return True

    # 가공 경로 평면과 STL actual_path삼각형의 교차점 추출
    def get_plane_intersection(self, model, order, N, a, path):
        N_p = model.normals[order]
        N_pn = N_p / math.sqrt(N_p[0] ** 2 + N_p[1] ** 2 + N_p[2] ** 2)
        p0 = model.v0[order]
        p1 = model.v1[order]
        p2 = model.v2[order]

        #print(self.cal_angle_acc(model.normals[order]))

        cross_dots = np.empty((0, 3), dtype = np.float32)
        if ((N_pn * N)[0] ** 2 + (N_pn * N)[1] ** 2 + (N_pn * N)[2] ** 2) != 1.0:
            # Line20의 교차점
            if self.check_intersection(p2, p0, a, N):
                s = abs(p2[2] - a[2]) / abs(p2[2] - p0[2])
                C20 = np.array([p2 + s * (p0 - p2)])

                cross_dots = np.append(cross_dots, C20, axis = 0)

            # Line12의 교차점
            if self.check_intersection(p1, p2, a, N):
                s = abs(p1[2] - a[2]) / abs(p1[2] - p2[2])
                C12 = np.array([p1 + s * (p2 - p1)])

                cross_dots = np.append(cross_dots, C12, axis = 0)

            # Line01의 교차점
            if self.check_intersection(p0, p1, a, N):
                s = abs(p0[2] - a[2]) / abs(p0[2] - p1[2])
                C01 = np.array([p0 + s * (p1 - p0)])

                cross_dots = np.append(cross_dots, C01, axis = 0) 

            if order == 0:
                return cross_dots, np.array([N_pn]) #self.cal_angle_acc(model.normals[order])

            else:
                return np.delete(cross_dots, 0, axis = 0), np.array([N_pn]) #self.cal_angle_acc(model.normals[order])

        else:
            return None, None

    def cal_angle_acc(self, normal):
        # normal shape: (N, 3)
        
        # 1. 단위 벡터화 (Normalize)
        norm = np.linalg.norm(normal, axis=1, keepdims=True)
        target_z_vecs = normal / norm
        
        euler_angles = []
        
        # 기준 벡터 설정 (월드 Z축인 (0,0,1)을 "Tool의 윗방향(Up Vector)"으로 가정하지 않고,
        # 가공 진행 방향이나 수직축을 고려해야 하지만, 가장 일반적인 방법은 아래와 같습니다)
        
        for z_vec in target_z_vecs:  
            # 만약 z_axis가 완벽한 수직(0,0,1)이라서 외적 불가할 경우 예외처리
            if np.isclose(abs(z_axis[2]), 1.0):
                x_axis = np.array([-1, 0, 0]) # 임의 설정
            else:
                x_axis = np.cross(np.array([0, 0, 1]), z_axis)
                x_axis = x_axis / np.linalg.norm(x_axis)
                
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            # 회전 행렬 구성 (Rotation Matrix 3x3)
            # R = [X_col, Y_col, Z_col]
            R_mat = np.column_stack((-x_axis, -y_axis, z_axis))
            
            # 행렬을 오일러 각으로 변환 (Extrinsic xyz)
            r = R.from_matrix(R_mat)
            euler = r.as_euler('xyz', degrees=False)
            euler_angles.append(euler)

        return np.array(euler_angles)

    # 3D 시각화
    def visualization(self, model, path, degs):
        # 가공 물체 시각화
        fig = plt.figure()
        ax = mplot3d.Axes3D(fig)
        poly_collection = mplot3d.art3d.Poly3DCollection(model.vectors)
        poly_collection.set_color((1.0,0.7,0.7))  # play with color
        ax.add_collection3d(poly_collection)
        scale = model.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)

        #for i in range(len(path)):
        #    path[i] = path[i] * 1000

        #가공 경로 시각화
        fig2 = plt.figure()
        ax2 = mplot3d.Axes3D(fig2)
        ax2.plot(path[:, 0], path[:, 1], zs = path[:, 2])
        ax2.scatter(path[:, 0], path[:, 1], path[:, 2])

        ax2.plot(self.robot_path[:, 0], self.robot_path[:, 1], zs = self.robot_path[:, 2])
        ax2.scatter(self.robot_path[:, 0], self.robot_path[:, 1], self.robot_path[:, 2])

        """for i in range(len(path)):
            #print(degs[i])
            ax2.plot([path[i, 0], self.robot_path[i, 0]], [path[i, 1],  self.robot_path[i, 1]], [path[i, 2], self.robot_path[i, 2]])"""

        """for i in range(len(path)):
            #print(degs[i])
            ax2.plot([path[i, 0], path[i, 0] - 10 * self.direction[i, 0]], [path[i, 1],  path[i, 1] - 10 * self.direction[i, 1]], [path[i, 2], path[i, 2] - 10 * self.direction[i, 2]])
        """
        """for i in range(len(path)):
            #print(degs[i])
            ax2.plot([self.robot_path[i, 0], self.robot_path[i, 0] - 10 * -math.sin(self.robot_path[i, 4])], [self.robot_path[i, 1],  self.robot_path[i, 1] - 10 * math.sin(self.robot_path[i, 5]) * math.cos(self.robot_path[i, 4])], [self.robot_path[i, 2], self.robot_path[i, 2] - 10 * math.cos(self.robot_path[i, 4]) * math.cos(self.robot_path[i, 5])])
        """
        ax2.auto_scale_xyz(scale, scale, scale)
        plt.show()

    def manual_move(self, key):
        if key == keyboard.KeyCode(char='u'):
            command = "{},{},0.4,180.0,0.0,90.0,X {},{},0.4,90.0,0.0,90.0,X {},{},{},90.0,0.0,90.0,X".format(self.trajectory[0][0], self.trajectory[0][1], self.trajectory[0][0], self.trajectory[0][1], self.trajectory[0][0], self.trajectory[0][1], self.trajectory[0][2])

            self.str_pub_pos.publish(command)
            rospy.sleep(7.0)

            for i in range(len(self.trajectory)):
                if i == 0:
                    command = "{},{},{},{},{},{},X".format(self.robot_path[0], self.robot_path[1], self.robot_path[2], self.robot_path[3], sself.robot_path[4], self.robot_path[5])
                else:        
                    command = command + " {},{},{},{},{},{},X".format(self.robot_path[0], self.robot_path[1], self.robot_path[2], self.robot_path[3], sself.robot_path[4], self.robot_path[5])
    
            self.str_pub_pos.publish(command)

    def _init_transform(self):
        """
        초기 1회 실행: 플랜지와 엔드 이펙터 사이의 '고정된' 관계를 구합니다.
        핵심: Flange -> EE가 아니라, EE -> Flange 관계를 미리 계산해 둡니다.
        """
        try:
            # 1. tool0(플랜지) -> ee_link(엔드 이펙터) 변환 조회
            (trans, rot) = self.listener.lookupTransform('ur_tool0', 'ee_link', rospy.Time(0))
            #trans = [i * 1000 for i in trans]
            
            # 2. 행렬로 변환 (T_flange_ee)
            mat_flange_ee = tf.transformations.compose_matrix(
                translate=trans,
                angles=tf.transformations.euler_from_quaternion(rot)
            )

            # 3. 역행렬 계산 (T_ee_flange)
            # 이것이 "엔드 이펙터 기준에서 플랜지가 어디에 있는가?"를 나타내는 행렬입니다.
            self.mat_ee_to_flange = np.linalg.inv(mat_flange_ee)
            self.is_ready = True
            rospy.loginfo("Transform initialized: EE -> Flange offset calculcated.")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF Init Failed: {e}")

    def get_flange_target_array(self, ee_target_array):
        """
        Args:
            ee_target_array: [x, y, z, roll, pitch, yaw] (Base 기준 엔드 이펙터 목표)
        Returns:
            flange_target_array: [x, y, z, roll, pitch, yaw] (Base 기준 플랜지 목표)
        """
        if not self.is_ready:
            rospy.logwarn("Transform not ready yet.")
            return None

        # 1. 입력받은 목표(EE Goal)를 4x4 행렬로 변환 (T_base_goal)
        goal_pos = ee_target_array[:3] / 1000
        goal_rpy = ee_target_array[3:]
        #print(goal_rpy)
        
        mat_base_goal = tf.transformations.compose_matrix(
            translate=goal_pos,
            angles=goal_rpy
        )

        # 2. 핵심 행렬 연산
        # 수식: T_base_flange = T_base_goal * T_ee_flange
        # 설명: 목표 지점의 회전(Orientation)이 T_ee_flange 위치 벡터를 회전시킵니다.
        #      이 과정에서 플랜지는 엔드 이펙터 회전에 따라 '바깥으로' 이동하게 됩니다.
        mat_base_flange = np.dot(mat_base_goal, self.mat_ee_to_flange)

        # 3. 결과 행렬 분해
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(mat_base_flange)
        #print(trans, angles)
        
        # [x, y, z, r, p, y] 리스트 생성
        return np.array([[i * 1000 for i in list(trans)] + list(angles)])

    def finalize(self):
        rospy.sleep(0.5)

    def main(self):
        self.path_planning(self.object)

        # Event handler for Keyboard Input 
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

if __name__ == "__main__":
    rospy.init_node('stl_tool_path')
    try:
        tool_path = STLToolPath()
        tool_path.main()

    except rospy.ROSInterruptException: pass