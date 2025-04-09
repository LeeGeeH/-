#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import rospy
from ackermann_msgs.msg import AckermannDriveStamped  # Ackermann 조향을 위한 메시지 타입
import time
from horse_power_sensor import HPSensor  # 센서 관련 사용자 정의 클래스

from controller import Stanley  # Stanley 제어 알고리즘
from lane_detector import LaneDetector  # 차선 인식 클래스
from obstacle_detector import Clustering  # 장애물 인식 및 군집화 클래스
from fsm import FiniteStateMachine # 상태전이 클래스

class HP:
    def __init__(self):
        self.rate = rospy.Rate(10)  # 루프 주기 설정: 10Hz
        self.motor_msg = AckermannDriveStamped()  # 보낼 메시지 객체 초기화
        self.motor_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=20)  # 속도/조향 명령을 보낼 퍼블리셔
        
        self.sensor = HPSensor()  # 센서 객체 생성
        self.sensor.init(self.rate)  # 센서 초기화 (카메라, 라이다 등)
       
        self.lane_detector = LaneDetector()  # 차선 검출기 객체 생성
        self.obstacle_detector = Clustering()  # 장애물 검출 객체 생성
        self.stanley = Stanley()  # Stanley 제어기 초기화

        # 주행 관련 변수 초기화
        self.speed = 35  # 기본 속도 설정
        self.steering_angle = 0
        self.calc_angle = 0

        # 장애물 관련 플래그 및 타이머
        self.obflag = False
        self.obs_time = None
        self.st_time = None

    def calc_speed(self, angle):
        """
        조향각(angle)에 따라 속도를 계산하는 함수
        조향각이 크면 속도를 줄이고, 작으면 속도를 높임
        """
        if angle > 0:
            slope = -0.7
        elif angle < 0:
            slope = 0.7
        else:
            slope = 0.0

        speed = (slope * angle) + 30.0  # 선형 회귀 방식으로 속도 결정
        return speed

    def control(self):
        """
        장애물 회피 및 차선 주행을 수행하는 메인 제어 함수
        """
        try:
            # 주행 시작 직후 조향 각도 급변 문제 방지를 위해 1초간 정속 주행
            if self.st_time is None:
                self.st_time = time.time()
                self.motor_msg.drive.speed = 10
                self.motor_msg.drive.steering_angle = 0

            # 장애물 검출 및 회피 각도 계산
            calc_angle = self.obstacle_detector.process(self.sensor.lidar_filtered)
            self.calc_angle = int(calc_angle)

            # 장애물 회피 상태에 들어갈 때 처리
            if self.obstacle_detector.fsm.current_state in ["AvoidRight", "AvoidLeft"]:
                if self.obs_time is None:
                    self.obs_time = time.time()  # 회피 시작 시간 저장
                    self.motor_msg.drive.speed = 0  # 정지
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle  # 후진 대비 조향
                    self.obflag = True
                    print("No.0")

            # 장애물 회피 중일 때 시간에 따라 동작 변경
            if self.obflag == True:

                if time.time() - self.st_time <= 1.0:
                    self.motor_msg.drive.speed = 0
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle
                    print("No.1")

                elif time.time() - self.obs_time <= 4.0:
                    self.motor_msg.drive.steering_angle = -1.5 * self.calc_angle
                    self.motor_msg.drive.speed = -15  # 후진
                    print("No.2")

                elif time.time() - self.obs_time <= 5.0:
                    self.motor_msg.drive.steering_angle = self.calc_angle
                    self.motor_msg.drive.speed = 0  # 정지
                    print("No.3")

                elif time.time() - self.obs_time <= 6.0:
                    self.motor_msg.drive.steering_angle = self.calc_angle
                    self.motor_msg.drive.speed = 40  # 전진
                    print("No.4")

                elif time.time() - self.obs_time <= 7.0:
                    self.motor_msg.drive.steering_angle = 0
                    self.motor_msg.drive.speed = 40  # 정상 주행 복귀
                    self.obflag = False
                    self.obs_time = None
                    print("No.5")

            self.motor_pub.publish(self.motor_msg)  # 조향/속도 명령 전송

        except ValueError:
            # 라이다 인식 실패 시 차선 주행 모드 진입
            if self.obflag == True:
                if time.time() - self.obs_time <= 1.0:
                    self.motor_msg.drive.speed = 0
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle
                    print("No.1")

                elif time.time() - self.obs_time <= 4.0:
                    self.motor_msg.drive.steering_angle = -1.5 * self.calc_angle
                    self.motor_msg.drive.speed = -15
                    print("No.2")

                elif time.time() - self.obs_time <= 5.0:
                    self.motor_msg.drive.steering_angle = self.calc_angle
                    self.motor_msg.drive.speed = 0
                    print("No.3")

                elif time.time() - self.obs_time <= 6.0:
                    self.motor_msg.drive.steering_angle = self.calc_angle
                    self.motor_msg.drive.speed = 40
                    print("No.4")

                elif time.time() - self.obs_time <= 7.0:
                    self.motor_msg.drive.steering_angle = 0
                    self.motor_msg.drive.speed = 40
                    self.obflag = False
                    self.obs_time = None
                    print("No.5")

            else:
                # 장애물이 없을 경우 차선 주행 수행
                self.obstacle_detector.fsm.transition(False)  # FSM 상태 초기화

                curvature_angle = self.lane_detector.process(self.sensor.cam)  # 차선 곡률 계산
                steering_angle = self.stanley.control(curvature_angle)  # Stanley 제어기 호출

                self.motor_msg.drive.speed = int(self.speed)  # 기본 속도 유지
                self.motor_msg.drive.steering_angle = int(steering_angle)  # 조향각 설정

                print("Current motor speed: {}, Current motor angle: {}".format(
                    self.motor_msg.drive.speed, self.motor_msg.drive.steering_angle))

            self.motor_pub.publish(self.motor_msg)  # 모터 명령 퍼블리시
            self.rate.sleep()  # 루프 주기 유지
