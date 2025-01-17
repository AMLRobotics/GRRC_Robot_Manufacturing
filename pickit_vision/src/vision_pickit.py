#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import sensor_msgs.msg
#import ros_numpy
import open3d as o3d
from cv_bridge import CvBridge, CvBridgeError
from pynput import keyboard
import os
import ctypes

class PKVision():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/CAM_30638/rgb/image_rect_color', sensor_msgs.msg.Image, self.image_callback)

        with keyboard.Listener(
            on_release=self.on_release) as listener:
            listener.join()

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.finalize()
            return False

    def finalize(self):
        rospy.sleep(0.5)
    
    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            cv2.namedWindow('test', cv2.WINDOW_NORMAL)
            cv2.imshow('test', self.cv_image)
            cv2.waitKey(1)
            
        except CvBridgeError as e:
            print(e)

if __name__ == "__main__":
    rospy.init_node('vision_pickit')
    try:
        pkvision = PKVision()
    except rospy.ROSInterruptException: pass