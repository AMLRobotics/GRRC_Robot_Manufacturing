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

class STLToolPath():
    def __init__(self):
        self.tool_height = float(rospy.get_param('/stl_tool_path/tool_diameter'))
        self.str_pub_pos = rospy.Publisher("str_pos", std_msgs.msg.String, queue_size = 10)

        self.trajectory = np.array([[]])
        self.degrees = np.array([[]])

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

        for i in range(int(self.z_max / self.tool_height) - 1):
            a = np.array([0, 0, (i + 1) * self.tool_height])
            
            for j in range(0, len(model.points)):
                dots, degs = self.get_plane_intersection(model, j, N, a, path)
            
                if dots is not None:
                    path = np.append(path, dots, axis = 0)
                    #print(degs)
                    direction = np.append(direction, degs, axis = 0)

            #GRRC 발표용 임시 코드
            for i in range(0, 43):
                path = np.delete(path, 0, axis = 0)
                direction = np.delete(direction, 0, axis = 0)

            x_rot = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
            z_rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

            for i in range(0, len(path)):
                path[i][0] = path[i][0] - 150
                path[i][1] = path[i][1] - 75
                path[i] = np.dot(path[i] , x_rot)
                #path[i] = np.dot(path[i] , z_rot)

                if i != 0:
                    temp = direction[i - 1][1]
                    direction[i - 1][1] = direction[i - 1][2]
                    direction[i - 1][2] = temp
                    #direction[i - 1] = -direction[i - 1]

                path[i][1] = path[i][1] - 450 - 38   
                path[i][2] = path[i][2] + 124 + 75

                path[i] = path[i] / 1000

            for i in range(0, 1 + int(len(direction) / 2)):
                direction[i] = direction[i] + direction[i + int(len(direction) / 2)] * 2
                
            for i in range(0, len(direction)):
                direction[i][1] = -(direction[i][1] + 90 - 360)

                direction[i][0] = direction[i][1]
                direction[i][1] = 0.0
                direction[i][2] = 90.0

            for i in range(0, len(path)):
                if i == 0:
                    path[i][0] = path[i][0] + 0.09 * math.cos(math.radians(-(direction[i][0] + 90 - 360)))
                    path[i][2] = path[i][2] + 0.09 * math.sin(math.radians(-(direction[i][0] + 90 - 360)))

                else:
                    path[i][0] = path[i][0] + 0.09 * math.cos(math.radians(-(direction[i - 1][0] + 90 - 360)))
                    path[i][2] = path[i][2] + 0.09 * math.sin(math.radians(-(direction[i - 1][0] + 90 - 360)))

        self.trajectory = path
        self.degrees = direction
        print(direction)

        #self.visualization(model, path, direction)

    # 가공 경로 평면과 교차점이 존재하는지 여부 확인
    def check_intersection(self, p, q, a, N):
        P_p = np.dot(N, p) - np.dot(N, a)
        P_q = np.dot(N, q) - np.dot(N, a)
        result = P_p * P_q

        if result > 0:
            return False

        else:
            return True

    # 가공 경로 평면과 STL 삼각형의 교차점 추출
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
                return cross_dots, self.cal_angle_acc(model.normals[order])

            else:
                return np.delete(cross_dots, 0, axis = 0), self.cal_angle_acc(model.normals[order])

        else:
            return None, None

    def cal_angle_acc(self, normal):

        N_n = normal / math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2)

        """y_radians = math.asin(N_n[2])
        z_radians = math.acos(N_n[1] / math.cos(y_radians))
        x_radians = 0.0"""
        
        z_radians = math.atan2(-normal[1], math.sqrt(math.pow(-normal[0], 2) + math.pow(-normal[2], 2)))
        y_radians = math.atan2(-normal[2], math.sqrt(math.pow(-normal[0], 2) + math.pow(-normal[1], 2)))
        x_radians = 0.0

        #print([x_radians, y_radians, z_radians])
        
        return np.array([[math.degrees(x_radians), math.degrees(y_radians), math.degrees(z_radians)]])
        #return np.array([N_n])

    # 3D 시각화
    def visualization(self, model, path, degs):
        # 가공 물체 시각화
        fig = plt.figure()
        ax = mplot3d.Axes3D(fig)
        poly_collection = mplot3d.art3d.Poly3DCollection(model.vectors)
        poly_collection.set_color((0.7,0.7,0.7))  # play with color
        ax.add_collection3d(poly_collection)
        scale = model.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)

        for i in range(len(path)):
            path[i] = path[i] * 1000
        
        #가공 경로 시각화
        fig2 = plt.figure()
        ax2 = mplot3d.Axes3D(fig2)
        ax2.plot(path[:, 0], path[:, 1], zs = path[:, 2])
        ax2.scatter(path[:, 0], path[:, 1], path[:, 2])

        """for i in range(len(path)):
            if i == 0:
                ax2.plot([path[i][0], path[i][0] - 10 * degs[i][0]], [path[i][1], path[i][1] - 10 * degs[i][1]], zs = [path[i][2], path[i][2] - 10 * degs[i][2]])
            
            else:
                ax2.plot([path[i][0], path[i][0] - 10 * degs[i - 1][0]], [path[i][1], path[i][1] - 10 * degs[i - 1][1]], zs = [path[i][2], path[i][2] - 10 * degs[i - 1][2]])"""

        ax2.auto_scale_xyz(scale, scale, scale)
        plt.show()

    def manual_move(self, key):
        if key == keyboard.KeyCode(char='u'):
            commend = "{},{},0.4,180.0,0.0,90.0,X {},{},0.4,90.0,0.0,90.0,X".format(self.trajectory[0][0], self.trajectory[0][1], self.trajectory[0][0], self.trajectory[0][1])

            for i in range(len(self.trajectory)):
                if i == 0:
                    command = commend + ' ' + "{},{},{},{},{},{},X".format(self.trajectory[i][0], self.trajectory[i][1], self.trajectory[i][2], self.degrees[i][0], self.degrees[i][1], self.degrees[i][2])
                else:        
                    command = command + ' ' + "{},{},{},{},{},{},X".format(self.trajectory[i][0], self.trajectory[i][1], self.trajectory[i][2], self.degrees[i - 1][0], self.degrees[i - 1][1], self.degrees[i -1][2])
       
            self.str_pub_pos.publish(command)

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