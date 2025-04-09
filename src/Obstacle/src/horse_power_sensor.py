#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# rospy: ROS Python 라이브러리
import rospy
# numpy: 수치 계산 라이브러리 (이 코드에서는 아직 사용되지 않음)
import numpy as np
# CvBridge: ROS 이미지 메시지를 OpenCV 이미지로 변환하는 브리지
from cv_bridge import CvBridge
# ROS에서 사용하는 센서 메시지 타입들
from sensor_msgs.msg import Image, LaserScan
# 초음파 센서 데이터를 담는 메시지 타입 (정수 배열)
from std_msgs.msg import Int32MultiArray

class HPSensor:
    def __init__(self):

        # 영상 데이터 초기화
        self.cam = None
        # 이미지 메시지 변환을 위한 CvBridge 객체 생성
        self.bridge = CvBridge()
        # 카메라 이미지 구독 (OpenCV용)
        rospy.Subscriber("/camera0/usb_cam/image_raw", Image, self.callback_cam)

        # 필터링된 라이다 데이터 저장 변수
        self.lidar_filtered = None
        # 필터링된 라이다 데이터 구독
        rospy.Subscriber("scan_filtered", LaserScan, self.callback_lidar_filtered)

    
    # 일반 카메라 콜백 함수
    def callback_cam(self, msg):
        self.cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # 필터링된 라이다 콜백 함수
    def callback_lidar_filtered(self, msg):
        self.lidar_filtered = msg

    # 센서가 모두 초기화될 때까지 기다리는 함수
    def init(self, rate):
        # 카메라 데이터가 수신될 때까지 대기
        while self.cam is None:
            rate.sleep()
        rospy.loginfo("video ready")  # 준비 완료 로그 출력

        # 필터링된 라이다 데이터가 수신될 때까지 대기
        while self.lidar_filtered is None:
            rate.sleep()
        rospy.loginfo("filtered lidar ready")
