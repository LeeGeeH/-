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
        # 초음파 센서 데이터 초기값 (리스트 형태)
        self.ultra = [0]
        # 초음파 센서 데이터 구독
        rospy.Subscriber("ultrasonic", Int32MultiArray, self.callback_ultra, queue_size=1)

        # 영상 데이터 초기화
        self.cam = None
        # 이미지 메시지 변환을 위한 CvBridge 객체 생성
        self.bridge = CvBridge()
        # 카메라 이미지 구독 (OpenCV용)
        rospy.Subscriber("/camera0/usb_cam/image_raw", Image, self.callback_cam)

        # 실제 영상 (real_cam) 변수 초기화
        self.real_cam = None
        # 동일한 이미지 토픽을 또 구독 (다른 목적, 예: 실제 처리용)
        rospy.Subscriber("/camera0/usb_cam/image_raw", Image, self.callback_real_cam)

        # 필터링된 라이다 데이터 저장 변수
        self.lidar_filtered = None
        # 필터링된 라이다 데이터 구독
        rospy.Subscriber("scan_filtered", LaserScan, self.callback_lidar_filtered)

    # 실제 카메라 콜백 함수
    def callback_real_cam(self, msg):
        # ROS 이미지 메시지를 OpenCV 형식으로 변환
        self.real_cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # 일반 카메라 콜백 함수
    def callback_cam(self, msg):
        self.cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # 필터링된 라이다 콜백 함수
    def callback_lidar_filtered(self, msg):
        self.lidar_filtered = msg
        # 예전 코드에서 라이다 길이를 출력하려고 했던 흔적 주석
        # print('rm', len(LaserScan.ranges))

    # 초음파 센서 콜백 함수
    def callback_ultra(self, msg):
        self.ultra = msg.data  # 수신된 데이터를 리스트로 저장
        print(list(self.ultra))  # 초음파 센서 값 출력

    # 센서가 모두 초기화될 때까지 기다리는 함수
    def init(self, rate):
        # 카메라 데이터가 수신될 때까지 대기
        while self.cam is None:
            rate.sleep()
        rospy.loginfo("video ready")  # 준비 완료 로그 출력

        # 실제 카메라 데이터가 수신될 때까지 대기
        while self.real_cam is None:
            rate.sleep()
        rospy.loginfo("usb_cam ready")

        # 필터링된 라이다 데이터가 수신될 때까지 대기
        while self.lidar_filtered is None:
            rate.sleep()
        rospy.loginfo("filtered lidar ready")

        # 초음파 데이터가 수신될 때까지 대기
        while self.ultra is None:
            rate.sleep()
        rospy.loginfo("ultra ready")
