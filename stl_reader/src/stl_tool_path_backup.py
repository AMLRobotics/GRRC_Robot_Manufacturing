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
import kdl_parser_py.urdf as urdf
import PyKDL as kdl
from stl_reader.srv import *
from scipy.optimize import minimize
import tf.transformations as tf_trans
from stl_reader.msg import *
from numpy.linalg import inv
from tf.transformations import euler_matrix, compose_matrix, translation_from_matrix, euler_from_matrix

class STLToolPath():
    def __init__(self):
        self.tool_height = float(rospy.get_param('/stl_tool_path/tool_diameter'))
        self.str_pub_pos = rospy.Publisher("str_pos", std_msgs.msg.String, queue_size = 10)
        self.pub_joint = rospy.Publisher("/ur_joint_deg_command", TargetJointDegs, queue_size = 10)
        self._init_kdl_solver()

        # 가공할 모델의 STL 파일 불러오기
        path = "/root/catkin_ws/src/stl_reader/"
        os.chdir(path)
        self.object = mesh.Mesh.from_file(rospy.get_param('/stl_tool_path/target_filename'))

        # A. base_link -> base 변환 (URDF: Z축 180도)
        self.T_base_link_to_base = euler_matrix(0, 0, math.pi)
        # 우리가 공식에서 필요한 건 (T_base^base_link)의 역행렬, 
        # 즉 T_base_to_base_link 이므로 역행렬을 취합니다.
        self.T_B_to_BL_inv = inv(self.T_base_link_to_base) 

        # B. flange -> tool0 변환 (URDF: X축 90도, Z축 90도)
        self.T_flange_to_tool0 = euler_matrix(math.pi/2, 0, math.pi/2)
        # 우리가 공식에서 필요한 건 (T_flange^tool0)의 역행렬입니다.
        self.T_F_to_T0_inv = inv(self.T_flange_to_tool0)

        print("FK 서비스 프록시를 초기화합니다...")
        try:
            rospy.wait_for_service('/FK_solve', timeout=5.0) # 무한 대기 방지
            # 클래스 인스턴스 변수로 저장하는 것이 좋습니다.
            self._fk_client = rospy.ServiceProxy('/FK_solve', joint2cartesian) 
        except rospy.ROSException:
            rospy.logerr("FK 서비스를 찾을 수 없습니다. 프로그램을 종료합니다.")
            return

        # 모델의 최대높이 측정
        self.z_max = 0.0
        for i in range(len(self.object.points)):
            if self.object.v0[i][2] > self.z_max:
                self.z_max = self.object.v0[i][2]

            if self.object.v1[i][2] > self.z_max:
                self.z_max = self.object.v1[i][2]

            if self.object.v2[i][2] > self.z_max:
                self.z_max = self.object.v2[i][2]

        self.trajectory = np.array([[]])
        self.degrees = np.array([[]])
        self.vectors = np.array([[]])

        self.listener = tf.TransformListener()
        self.robot_path = np.array([[]])
        self.mat_flange_to_ee = None
        self.is_ready = False
        self.path_layer = int(self.z_max / (self.tool_height / 2))
        self.action = 0

        # TF 데이터가 들어올 때까지 잠시 대기
        rospy.sleep(1.0)
        self._init_transform()
        self.base_to_object = self.get_ur_base_to_object_base()
        #print(self.base_to_object)

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

        full_trajectory = []
        full_degrees = []
        full_vectors = []

        for i in range(self.path_layer):
            #상단 평면 경로 작성
            if i == self.path_layer - 1:
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

                    top_trajectory = np.append(top_trajectory, next_line.copy(), axis = 0)

                top_direction = np.repeat(np.array([[0, 0, 1.0]]), len(top_trajectory), axis = 0)
                top_degs= self.cal_angle_acc(-top_direction) 
            
            #측면 경로 작성
            else:
                # 높이별 반복 (Side Path)
                a = np.array([0, 0, (i + 1) * self.tool_height / 2])
                z_height = (i + 1) * self.tool_height / 2
                
                # 해당 높이에서의 교차점들을 모두 찾습니다.
                layer_dots = np.empty((0, 3), dtype=np.float32)
                layer_degs = np.empty((0, 3), dtype=np.float32)

                # 2. 옆면(Side) 경로 생성 - 4개 섹터로 분할 필요 X
                layer_dots, layer_vecs = self.get_points_for_height(z_height, 0, 2 * np.pi)
                layer_degs = self.cal_angle_acc(-layer_vecs) 
            
                if len(layer_dots) > 0:
                    # 중심점(cx, cy) 계산
                    cx = np.mean(layer_dots[:, 0])
                    cy = np.mean(layer_dots[:, 1])

                    # 정렬을 위한 각도 계산 (-pi ~ pi)
                    angles = np.arctan2(layer_dots[:, 1] - cy, layer_dots[:, 0] - cx)
                    
                    # 각도 순으로 정렬 (Sorting: 0도에서 360도 방향으로 정렬됨)
                    sort_idx = np.argsort(angles)
                    sorted_dots = layer_dots[sort_idx]
                    sorted_degs = layer_degs[sort_idx]   
                    sorted_vecs = layer_vecs[sort_idx]

                    # --- [핵심] 지그재그(Zig-Zag) 연결 ---
                    # 짝수 층(0, 2, 4...)은 정방향 가공
                    # 홀수 층(1, 3, 5...)은 역방향 가공 (배열을 뒤집음)
                    # 이렇게 해야 로봇 관절이 한 방향으로 무한히 감기지 않습니다.
                    if i % 2 == 1:
                        sorted_dots = np.flip(sorted_dots, axis=0)
                        sorted_degs = np.flip(sorted_degs, axis=0)
                        sorted_vecs = np.flip(sorted_vecs, axis=0)
                    
                    # 정렬된 해당 층의 데이터를 전체 궤적 리스트에 추가
                    full_trajectory.append(sorted_dots)
                    full_degrees.append(sorted_degs)
                    full_vectors.append(sorted_vecs)

                # 2. 모든 층의 궤적을 하나의 numpy 배열로 병합
                if len(full_trajectory) > 0:
                    self.trajectory = np.concatenate(full_trajectory, axis=0)
                    self.degrees = np.concatenate(full_degrees, axis=0)
                    self.vectors = np.concatenate(full_vectors, axis=0)                    
                else:
                    self.trajectory = np.empty((0, 3))
                    self.degrees = np.empty((0, 3))
                    print("경고: 생성된 궤적이 없습니다.")
                    return

        self.degrees = np.append(self.degrees, top_degs, axis = 0)
        self.trajectory = np.append(self.trajectory, top_trajectory, axis = 0)
        self.vectors = np.append(self.vectors, top_direction, axis = 0)

        self.robot_path = np.empty((0, 6), dtype=float) # 초기화 방식 수정 권장
        self.robot_angle = np.empty((len(self.trajectory), 3), dtype=float)

        robot_path_list = []
        for i in range(len(self.trajectory)):
            # 1. 위치
            pos_x = self.trajectory[i][0]
            pos_y = self.trajectory[i][1]
            pos_z = self.trajectory[i][2]
            
            r = R.from_euler('xyz', self.degrees[i], degrees=False)
            rot_vec = r.as_rotvec()
            self.robot_angle[i] = rot_vec

            # get_flange_target_array가 numpy array를 반환한다고 가정
            flange_pose = self.get_flange_target_array(np.array([
                self.trajectory[i][0], self.trajectory[i][1], self.trajectory[i][2], 
                self.degrees[i][0], self.degrees[i][1], self.degrees[i][2]
            ]))
            if flange_pose is not None:
                 # get_flange_target_array의 리턴 형태에 따라 [0] 인덱싱 필요 여부 확인
                 # 보통 (1, 6) shape이면 flatten 필요
                 robot_path_list.append(flange_pose.flatten()) 

        self.robot_path = np.array(robot_path_list)

        for i in range(len(self.vectors)):
            norm = np.linalg.norm(self.vectors[i])
            if norm is not 0:
                self.vectors[i] = self.vectors[i] / norm

        # ---------------------------------------------------------
        # 3. [차동 적분 루프 적용 부분]
        # ---------------------------------------------------------
        print("최적화 기반 관절 궤적 적분을 시작합니다. 시간이 다소 소요될 수 있습니다...")
        
        self.robot_joint_path = []
        
        # 가공 시작 전 로봇의 편안한 초기 관절 각도를 설정합니다. (단위: Radian)
        rospy.wait_for_service('/IK_solve')
        q = [-np.pi/2.0, -np.pi/2.0, -np.pi/2.0, -np.pi/2.0, np.pi/2.0, -np.pi]

        # 좌표계 맞추기
        b_to_t_pos = np.hstack([((self.robot_path[:, 0] - 50) / 1000 + self.base_to_object[0]).reshape(len(self.robot_path), 1), 
                                ((self.robot_path[:, 1] - 50) / 1000 + self.base_to_object[1]).reshape(len(self.robot_path), 1), 
                                (self.robot_path[:, 2] / 1000  + self.base_to_object[2]).reshape(len(self.robot_path), 1)])
        b_to_t_rot = self.robot_path[:, 3:]
        #print((self.robot_path[:, 0] - 50) / 1000 + self.base_to_object[0])
        target_trans = np.empty((0, 3), dtype = np.float32)
        target_rpy = np.empty((0, 3), dtype = np.float32)

        for i in range(len(b_to_t_pos)):
            # 1. [버그 수정] translate와 angles의 인자를 올바르게 매칭!
            T_b_to_t = compose_matrix(translate=b_to_t_pos[i], angles=b_to_t_rot[i])
            
            # 2. Base -> Base_link 및 Tool0 -> Flange 변환이 모두 적용된 완벽한 행렬
            T_bl_to_f = np.dot(self.T_B_to_BL_inv, np.dot(T_b_to_t, self.T_F_to_T0_inv))

            # 3. 위치 추출 (-X, -Y 수동 조작 대신 행렬에서 바로 추출하여 100% 안전)
            trans_bl = translation_from_matrix(T_bl_to_f)
            target_trans = np.append(target_trans, [trans_bl], axis = 0)
            target_rpy = np.append(target_rpy, np.array([euler_from_matrix(T_bl_to_f)]) * 180.0 / np.pi, axis = 0)

        #print(self.robot_angle * 180.0 / np.pi, target_rpy)
        try:
            solve_ik = rospy.ServiceProxy('/IK_solve', cartesian2joint)
            joints = solve_ik(target_trans[0][0], target_trans[0][1], target_trans[0][2], target_rpy[0][0], target_rpy[0][1], target_rpy[0][2], q)
            joints = np.array(joints.jointDegs)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        current_q = joints
        self.robot_joint_path.append(current_q)
            
        print("메인 궤적 생성을 시작합니다...")

        # 궤적을 따라가며 한 걸음씩 최적화 적분
        for i in range(len(self.robot_path) - 1):
            current_pos = np.array([target_trans[i][0], target_trans[i][1], target_trans[i][2]])
            next_pos = np.array([target_trans[i + 1][0], target_trans[i + 1][1], target_trans[i + 1][2]])
            current_dir = self.vectors[i]
            next_dir = self.vectors[i + 1]  

            try:
                # 2. FK 서비스 호출 (미리 생성된 self._fk_client 사용)
                # timeout=30은 너무 깁니다. 궤적 제어 루프는 ms 단위여야 합니다.
                # 필요하다면 짧게 주거나(0.01), C++ 노드가 충분히 빠르다고 믿고 생략합니다.
                actual_flange_pos = self._fk_client(current_q) 
                
                # 3. 서비스 응답 데이터를 배열로 변환
                actual_pos_array = np.array([actual_flange_pos.x, actual_flange_pos.y, actual_flange_pos.z, 
                                             actual_flange_pos.roll, actual_flange_pos.pitch, actual_flange_pos.yaw])

                mat_baselink_flange = tf_trans.compose_matrix(translate=actual_pos_array[:3], angles=actual_pos_array[3:])

            except rospy.ServiceException as e:
                print("FK 서비스 호출 실패: %s"%e)
                # 실패 시 폭주 방지를 위해 루프 탈출
                break

            R_z_180 = tf_trans.euler_matrix(0, 0, np.pi)
            #mat_base_flange = np.dot(R_z_180, mat_baselink_flange)
            _, _, _, trans, _ = tf_trans.decompose_matrix(mat_baselink_flange)

            tip_tcp_pos = np.array(trans)
            #if i == 0:
            #    print(current_pos, tip_tcp_pos, self.base_to_object[:3])

            Kp = 0.5
            # 위치 이동량 (dx, dy, dz)
            step_target = next_pos - current_pos
            error = current_pos - tip_tcp_pos  # 2. 지난 스텝까지 궤적을 벗어난 누적 오차
            delta_pos = step_target + (Kp * error)
                
                # 목표 지점이 완전히 동일하다면(제자리 이동 등) 최적화 건너뜀
            if np.linalg.norm(delta_pos) < 1e-5:
                self.robot_joint_path.append(current_q)
                continue
                    
            # 최적의 관절 미세 변화량(D_q) 계산
            delta_q = self.optimize_differential_step(current_q, delta_pos, current_dir, next_dir)
                
            # 관절 각도 업데이트 (적분 누적)
            next_q = current_q + delta_q
            self.robot_joint_path.append(next_q)
                
            current_q = next_q # 다음 스텝을 위해 갱신

        self.robot_joint_path = np.array(self.robot_joint_path)
        print("최적화 궤적 생성 완료! (전체 층 단일 가공 궤적)")
        #print(self.robot_joint_path)

        self.robot_cartesian_path = np.empty((0, 6), dtype=float)
        timeout = rospy.Duration(30)

        rospy.wait_for_service('/FK_solve')
        try:
            solve_fk = rospy.ServiceProxy('/FK_solve', joint2cartesian)
            for i in range(len(self.robot_joint_path)):
                cartesian_pos = solve_fk(self.robot_joint_path[i])
                solve_fk.wait_for_service(timeout)
                self.robot_cartesian_path = np.append(self.robot_cartesian_path, np.array([[cartesian_pos.x, cartesian_pos.y, cartesian_pos.z, cartesian_pos.roll, cartesian_pos.pitch, cartesian_pos.yaw]]), axis = 0)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        #print(self.robot_cartesian_path)

        # 시각화 실행
        self.visualization(model, self.trajectory, self.degrees)

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
            z_axis = z_vec
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

        #ax2.plot(self.robot_path[:, 0], self.robot_path[:, 1], zs = self.robot_path[:, 2] )
        #ax2.scatter(self.robot_path[:, 0], self.robot_path[:, 1], self.robot_path[:, 2])

        ax2.plot((self.robot_cartesian_path[:, 0] + 0.05 + self.base_to_object[0]) * 1000, (self.robot_cartesian_path[:, 1] + 0.05 + self.base_to_object[1]) * 1000, (self.robot_cartesian_path[:, 2] - self.base_to_object[2]) * 1000)
        ax2.scatter((self.robot_cartesian_path[:, 0] + 0.05 + self.base_to_object[0]) * 1000, (self.robot_cartesian_path[:, 1] + 0.05 + self.base_to_object[1]) * 1000, (self.robot_cartesian_path[:, 2] - self.base_to_object[2]) * 1000)
        #print(self.robot_cartesian_path[:, :3], self.robot_cartesian_path[:, 3:] * 180.0 / np.pi)

        mat_rp = R.from_euler('zyx', self.robot_cartesian_path[:, 3:])
        mat_rp = mat_rp.as_matrix()
        rp_vec = mat_rp[:, :, 2]
        print(rp_vec.shape)

        """for i in range(len(path)):
            #print(degs[i])
            ax2.plot([path[i, 0], self.robot_path[i, 0]], [path[i, 1],  self.robot_path[i, 1]], [path[i, 2], self.robot_path[i, 2]])"""

        """for i in range(len(path)):
            #print(degs[i])
            ax2.plot([path[i, 0], path[i, 0] - 10 * self.vectors[i, 0]], [path[i, 1],  path[i, 1] - 10 * self.vectors[i, 1]], [path[i, 2], path[i, 2] - 10 * self.vectors[i, 2]])
        """
        for i in range(len(path)):
            #print(degs[i])
            ax2.plot([(self.robot_cartesian_path[i, 0] + 0.05 + self.base_to_object[0]) * 1000, (self.robot_cartesian_path[i, 0] + 0.05 + self.base_to_object[0]) * 1000 - 10 * rp_vec[i, 0]], 
                    [(self.robot_cartesian_path[i, 1] + 0.05 + self.base_to_object[1]) * 1000, (self.robot_cartesian_path[i, 1] + 0.05 + self.base_to_object[1]) * 1000 - 10 * rp_vec[i, 1]],    
                    [(self.robot_cartesian_path[i, 2] - self.base_to_object[2]) * 1000, (self.robot_cartesian_path[i, 2] - self.base_to_object[2]) * 1000 - 10 * rp_vec[i, 2]])
        
        ax2.auto_scale_xyz(scale, scale, scale)
        plt.show()

    def manual_move(self, key):
        if key == keyboard.KeyCode(char='u'):
            if self.action == 0:
                command = "0.0,-0.3,0.4,3.14159,0.0,0.0,-2.0"
                command = command + " {},{},0.4,{},{},{},-2.0".format((self.robot_path[0, 0] - 50) / 1000 + self.base_to_object[0], 
                                                                    (self.robot_path[0, 1] - 50) / 1000 + self.base_to_object[1], 
                                                                    self.robot_angle[0, 0], 
                                                                    self.robot_angle[0, 1], 
                                                                    self.robot_angle[0, 2])
                command = command + " {},{},{},{},{},{},-2.0".format((self.robot_path[0, 0] - 50) / 1000 + self.base_to_object[0], 
                                                                    (self.robot_path[0, 1] - 50) / 1000 + self.base_to_object[1], 
                                                                    self.robot_path[0, 2] / 1000 + self.base_to_object[2], 
                                                                    self.robot_angle[0, 0], 
                                                                    self.robot_angle[0, 1], 
                                                                    self.robot_angle[0, 2])
                           
                self.str_pub_pos.publish(command)
                #rospy.sleep(7.0)
                self.action = 1

            elif self.action == 1:
                for i in range(len(self.trajectory)):
                    if i == 0:
                        command = "{},{},{},{},{},{},-2.0".format((self.robot_path[i, 0] - 50 ) / 1000 + self.base_to_object[0],
                                                                (self.robot_path[i, 1] - 50) / 1000 + self.base_to_object[1],
                                                                self.robot_path[i, 2] / 1000 + self.base_to_object[2], 
                                                                self.robot_angle[i, 0], 
                                                                self.robot_angle[i, 1], 
                                                                self.robot_angle[i, 2])
                    else:        
                        command = command + " {},{},{},{},{},{},-2.0".format((self.robot_path[i, 0] - 50) / 1000 + self.base_to_object[0], 
                                                                            (self.robot_path[i, 1]- 50) / 1000 + self.base_to_object[1], 
                                                                            self.robot_path[i, 2] / 1000 + self.base_to_object[2], 
                                                                            self.robot_angle[i, 0], 
                                                                            self.robot_angle[i, 1], 
                                                                            self.robot_angle[i, 2])
        
                self.str_pub_pos.publish(command)
                self.action = 0

        if key == keyboard.KeyCode(char='i'):
            if self.action == 0:
                initial_command = np.array([[0.0, 0.3, 0.4, 0.0, 90.0, 270.0], 
                                            [-self.base_to_object[0], -self.base_to_object[1], 0.4, self.robot_cartesian_path[0][3], self.robot_cartesian_path[0][4], self.robot_cartesian_path[0][5]]]) 
                                            #[self.robot_cartesian_path[0][0], self.robot_cartesian_path[0][1], 0.4, self.robot_cartesian_path[0][3], self.robot_cartesian_path[0][4], self.robot_cartesian_path[0][5]]]) 
                                            #self.robot_cartesian_path[0]])
                
                print(initial_command)
                rospy.wait_for_service('/IK_solve')
                joint_list = np.empty((0, 6), dtype = np.float32)
                q = [-np.pi/2.0, -np.pi/2.0, -np.pi/2.0, -np.pi/2.0, np.pi/2.0, -np.pi]
                
                try:
                    for i in initial_command:
                        solve_ik = rospy.ServiceProxy('/IK_solve', cartesian2joint)
                        joints = solve_ik(i[0], i[1], i[2], i[3], i[4], i[5], q)
                        joints = np.array(joints.jointDegs)
                        q = joints
                        joint_list = np.append(joint_list, joints)

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)
        
                self.pub_joint.publish(joint_list)
                #rospy.sleep(7.0)
                self.action = 1

            elif self.action == 1:

                self.pub_joint.publish((self.robot_joint_path.reshape(len(self.robot_joint_path) * 6)).tolist())
                self.action = 0

    def get_ur_base_to_object_base(self):
        (trans, rot) = self.listener.lookupTransform('ur_base', 'object_base_link', rospy.Time(0))
        #print(trans, rot)
             
        return np.array(trans + list(tf.transformations.euler_from_quaternion(rot)))


    def _init_transform(self):
        """
        초기 1회 실행: 플랜지와 엔드 이펙터 사이의 '고정된' 관계를 구합니다.
        핵심: Flange -> EE가 아니라, EE -> Flange 관계를 미리 계산해 둡니다.
        """
        try:
            # 1. tool0(플랜지) -> ee_link(엔드 이펙터) 변환 조회
            (trans, rot) = self.listener.lookupTransform('ur_tool0', 'ur_ee_link', rospy.Time(0))
            #trans = [i * 1000 for i in trans]
             
            # 2. 행렬로 변환 (T_flange_ee)
            mat_flange_ee = tf.transformations.compose_matrix(
                translate=trans,
                angles=tf.transformations.euler_from_quaternion(rot)
            )

            # 3. 역행렬 계산 (T_ee_flange)
            # 이것이 "엔드 이펙터 기준에서 플랜지가 어디에 있는가?"를 나타내는 행렬입니다.
            self.mat_flange_to_ee = mat_flange_ee
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

    def get_ee_target_array(self, flange_target_array):
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
        goal_pos = flange_target_array[:3] / 1000
        goal_rpy = flange_target_array[3:]
        #print(goal_rpy)
        
        mat_base_flange = tf.transformations.compose_matrix(
            translate=goal_pos,
            angles=goal_rpy
        )

        # 2. 핵심 행렬 연산
        # 수식: T_base_flange = T_base_goal * T_ee_flange
        # 설명: 목표 지점의 회전(Orientation)이 T_ee_flange 위치 벡터를 회전시킵니다.
        #      이 과정에서 플랜지는 엔드 이펙터 회전에 따라 '바깥으로' 이동하게 됩니다.
        mat_base_ee = np.dot(mat_base_flange, self.mat_flange_to_ee)

        R_z_180 = tf_trans.euler_matrix(0, 0, np.pi)
        print(mat_base_flange, R_z_180)
        mat_base_flange = np.dot(mat_base_flange, R_z_180)
        print(mat_base_flange)

        # 3. 결과 행렬 분해
        scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(mat_base_flange)
        #print(trans, angles)
        
        # [x, y, z, r, p, y] 리스트 생성
        return np.array([[i * 1000 for i in list(trans)] + list(angles)])

    def get_points_for_height(self, z_height, start_ang, end_ang):
        """
        특정 Z 높이에서 STL 모델과 평면의 교차점을 찾고, 
        주어진 섹터 각도(start_ang ~ end_ang) 내에 있는 점들만 필터링하여 반환합니다.
        """
        N = np.array([0, 0, 1])
        a = np.array([0, 0, z_height])
        
        layer_dots = np.empty((0, 3), dtype=np.float32)
        layer_degs = np.empty((0, 3), dtype=np.float32)

        # 1. 해당 높이(z_height)의 모든 교차점 찾기 (사용자님의 기존 로직)
        for j in range(0, len(self.object.points)):
            dots, degs = self.get_plane_intersection(self.object, j, N, a, np.empty((0, 3)))
            if dots is not None:
                layer_dots = np.append(layer_dots, dots, axis=0)
                layer_degs = np.append(layer_degs, degs, axis=0)

        # 교차점이 아예 없으면 빈 배열 반환
        if len(layer_dots) == 0:
            return np.empty((0,3)), np.empty((0,3))

        # 2. [이전 오류 수정 반영] dots와 degs의 배열 길이 맞추기
        if len(layer_dots) > len(layer_degs):
            diff = len(layer_dots) - len(layer_degs)
            if len(layer_degs) > 0:
                extras = np.tile(layer_degs[-1], (diff, 1))
                layer_degs = np.append(layer_degs, extras, axis=0)
            else:
                layer_degs = np.append(layer_degs, np.array([[0, 0, 1]] * diff), axis=0)
        elif len(layer_dots) < len(layer_degs):
            layer_degs = layer_degs[:len(layer_dots)]

        # 3. 중심점 계산 및 각도 구하기
        center_x = np.mean(layer_dots[:, 0])
        center_y = np.mean(layer_dots[:, 1])
        raw_angles = np.arctan2(layer_dots[:, 1] - center_y, layer_dots[:, 0] - center_x)
        
        # 4. 각도를 0 ~ 2pi 범위로 정규화
        angles = (raw_angles + 2 * np.pi) % (2 * np.pi)

        # 5. 현재 섹터(Sector) 범위에 맞는 점들만 마스킹(필터링)
        if start_ang > end_ang:
            # 0도를 걸치는 섹터 (예: 315도 ~ 45도) -> OR 조건
            mask = (angles >= start_ang) | (angles <= end_ang)
        else:
            # 일반적인 섹터 (예: 45도 ~ 135도) -> AND 조건
            mask = (angles >= start_ang) & (angles <= end_ang)

        valid_dots = layer_dots[mask]
        valid_degs = layer_degs[mask]

        return valid_dots, valid_degs

    def optimize_differential_step(self, current_q, delta_pos_m, current_dir, next_dir):
        # ========================================================
        # 1. 좌표계 변환 (사용자님의 정답 유지)
        # ========================================================
        # delta_pos_m = np.array([-delta_pos_m[0], -delta_pos_m[1], delta_pos_m[2]])
        # current_dir = np.array([-current_dir[0], -current_dir[1], current_dir[2]])
        # next_dir = np.array([-next_dir[0], -next_dir[1], next_dir[2]])

        # ========================================================
        # 2. 필수 트위스트(T_req)와 자유 트위스트(T_free) 분리
        # ========================================================
        rot_align = np.cross(current_dir, next_dir)
        
        # 공구가 궤적을 따라가기 위해 반드시 움직여야 하는 필수 6D 벡터
        T_req = np.concatenate((delta_pos_m, rot_align))
        
        # 공구 축(Z축)을 중심으로 팽이처럼 도는 자유로운 6D 벡터
        T_free = np.concatenate(([0.0, 0.0, 0.0], next_dir))

        # ========================================================
        # 3. 안전한 야코비안 역행렬 계산 (DLS 유지 - 특이점 방어)
        # ========================================================
        J_current = self.get_jacobian(current_q)
        lambda_dls = 0.01 
        J_inv = J_current.T @ np.linalg.inv(J_current @ J_current.T + (lambda_dls**2) * np.eye(6))

        # 각각의 트위스트가 유발하는 관절 변화량 도출
        dq_req = np.dot(J_inv, T_req)
        dq_free = np.dot(J_inv, T_free)

        # ========================================================
        # 4. [수정] 순수 관절 속도 최소화를 위한 해석적 해 (Analytical Alpha)
        # 강제로 손목을 0도로 묶는 대신, "가장 관절을 덜 움직이는" 공구 회전량을 찾습니다.
        # W 행렬이나 q_ref가 필요 없는 가장 우아한 형태의 수식입니다.
        # 수학 모델: minimize || dq_req + alpha * dq_free ||^2
        # ========================================================
        
        # 두 벡터의 내적(Dot product)을 이용해 한 줄로 최적의 알파를 도출합니다.
        numerator = np.dot(dq_free.T, dq_req)
        denominator = np.dot(dq_free.T, dq_free)
        
        if denominator < 1e-6:
            alpha = 0.0 # 특이점에서 0으로 나누기 방지
        else:
            alpha = -numerator / denominator

        # 한 스텝당 너무 과도한 회전(예: 약 5.7도 이상)을 막기 위한 클리핑
        alpha = np.clip(alpha, -0.1, 0.1)

        # ========================================================
        # 5. 최종 관절 변화량 도출 및 속도 스케일링 (기존과 동일)
        # ========================================================
        opt_D_q = dq_req + alpha * dq_free
        
        max_step_norm = 0.18
        current_norm = np.linalg.norm(opt_D_q)
        if current_norm > max_step_norm:
            opt_D_q = opt_D_q * (max_step_norm / current_norm)

        return opt_D_q

    def get_jacobian(self, joint_angles):
        """
        주어진 관절 각도(joint_angles)에서 6x6 야코비안 Numpy 배열을 반환합니다.
        """
        # 1. Numpy 배열을 KDL JntArray로 변환
        for i in range(self.num_joints):
            self.kdl_jnt_array[i] = joint_angles[i]
            
        # 2. 야코비안 계산 (결과는 self.kdl_jacobian에 저장됨)
        self.jac_solver.JntToJac(self.kdl_jnt_array, self.kdl_jacobian)
        
        # 3. KDL 야코비안을 Numpy 6x6 배열로 변환
        J_np = np.zeros((6, self.num_joints))
        for i in range(6):
            for j in range(self.num_joints):
                J_np[i, j] = self.kdl_jacobian[i, j]
                
        return J_np      

    def _init_kdl_solver(self):
        """
        KDL 체인 및 야코비안 솔버를 초기화합니다. (프로그램 실행 시 1회만 호출)
        """
        # 1. 파라미터 서버에서 URDF 로드하여 트리 생성
        success, tree = urdf.treeFromParam("robot_description")
        if not success:
            raise RuntimeError("로봇 URDF를 로드할 수 없습니다. 'robot_description' 파라미터를 확인하세요.")
        
        # 2. 베이스 링크에서 툴팁 링크까지의 기구학 체인 추출
        # 주의: UR5의 실제 링크 이름에 맞춰 수정해야 할 수 있습니다. 
        # (예: 'base_link' -> 'tool0' 또는 'tcp')
        self.kdl_chain = tree.getChain("ur_base_link", "ur_tool0")
        
        # 3. 야코비안 솔버 생성
        self.jac_solver = kdl.ChainJntToJacSolver(self.kdl_chain)
        self.num_joints = self.kdl_chain.getNrOfJoints()
        
        # 4. 메모리 할당 (계산 속도 향상을 위해 미리 할당)
        self.kdl_jacobian = kdl.Jacobian(self.num_joints)
        self.kdl_jnt_array = kdl.JntArray(self.num_joints)

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