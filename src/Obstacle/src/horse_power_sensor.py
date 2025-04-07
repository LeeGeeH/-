#!/usr/bin/env python3
# -*- coding:utf-8 -*-

#from Tkinter import N
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray

class HPSensor:

    def __init__(self):
        self.ultra = [0]
        rospy.Subscriber("ultrasonic", Int32MultiArray,self.callback_ultra, queue_size=1)
        # video
        self.cam = None
        self.bridge = CvBridge()
        rospy.Subscriber("/camera0/usb_cam/image_raw", Image, self.callback_cam)

        # camera
        self.real_cam = None
        rospy.Subscriber("/camera0/usb_cam/image_raw", Image, self.callback_real_cam)

        # lidar filtered
        self.lidar_filtered = None
        rospy.Subscriber("scan_filtered", LaserScan, self.callback_lidar_filtered)

    def callback_real_cam(self, msg):
        self.real_cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_cam(self, msg):
        self.cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def callback_lidar_filtered(self, msg):
        self.lidar_filtered = msg
        # print('rm', len(LaserScan.ranges))

    def callback_ultra(self, msg):
        self.ultra = msg.data
        print(list(self.ultra))    


    def init(self, rate):
        while self.cam is None:
            rate.sleep()
        rospy.loginfo("video ready")

        while self.real_cam is None:
            rate.sleep()
        rospy.loginfo("usb_cam ready")

        while self.lidar_filtered is None:
            rate.sleep()
        rospy.loginfo("filtered lidar ready") 


        while self.ultra is None:
            rate.sleep()
        rospy.loginfo("ultra ready") 
