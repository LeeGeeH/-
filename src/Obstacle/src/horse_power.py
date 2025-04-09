#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# - Python3 실행을 위한 shebang 라인
# - UTF-8 인코딩 지정으로 한글 등 유니코드 문자 지원

import cv2  # OpenCV 라이브러리: 이미지 처리용 (현재 코드에서는 직접 사용되지 않음)
import rospy  # ROS Python 라이브러리: ROS 노드와 토픽 통신을 위한 핵심 모듈
from ackermann_msgs.msg import AckermannDriveStamped  # AckermannDriveStamped 메시지 타입: 조향각과 속도를 담는 ROS 메시지
import time  # 시간 관련 함수 제공: 타이밍 제어에 사용
from horse_power_sensor import HPSensor  # 사용자 정의 센서 클래스: 카메라와 LiDAR 데이터 수집
from controller import Stanley  # Stanley 제어 알고리즘: 차선 주행 시 조향각 보정
from lane_detector import LaneDetector  # 차선 인식 클래스: 카메라 이미지에서 차선 탐지
from obstacle_detector import Clustering  # 장애물 탐지 및 클러스터링 클래스: LiDAR 기반 장애물 회피
from fsm import FiniteStateMachine  # 유한 상태 머신(FSM): 장애물 회피 상태 관리

class HP:
    # HP 클래스: 자율 주행 차량의 전체 제어를 담당하는 핵심 클래스
    def __init__(self):
        # 객체 초기화 메서드: ROS 설정, 센서, 제어 모듈 초기화
        self.rate = rospy.Rate(10)  # ROS 루프 주기 설정: 10Hz (초당 10번 실행)
        self.motor_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=20)  
        # - ROS 퍼블리셔 객체 생성: '/ackermann_cmd' 토픽에 메시지 발행
        # - AckermannDriveStamped: 속도와 조향각을 담는 메시지 타입
        # - queue_size=20: 발행 대기열 크기 (20개 메시지까지 대기 가능)
        self.motor_msg = AckermannDriveStamped()  
        # - 실제로 보낼 메시지 객체 생성: 속도와 조향각 데이터를 채울 용도
        
        self.sensor = HPSensor()  # HPSensor 객체 생성: 카메라와 LiDAR 데이터 제공
        self.sensor.init(self.rate)  # 센서 초기화 호출: 모든 센서 데이터가 준비될 때까지 대기
        
        self.lane_detector = LaneDetector()  # 차선 탐지 객체 생성: 카메라 이미지로 차선 인식
        self.obstacle_detector = Clustering()  # 장애물 탐지 객체 생성: LiDAR로 장애물 회피
        self.stanley = Stanley()  # Stanley 제어기 객체 생성: 차선 주행 조향각 계산

        # 주행 관련 변수 초기화
        self.speed = 35  # 기본 속도 설정: 정상 주행 시 사용할 속도 (단위는 코드 맥락상 추정)
        self.steering_angle = 0  # 현재 조향각 초기값: 직진 상태
        self.calc_angle = 0  # 계산된 조향각 저장용: 장애물 회피용으로 사용

        # 장애물 회피 관련 변수 초기화
        self.obflag = False  # 장애물 회피 플래그: False면 비활성, True면 회피 동작 중
        self.obs_time = None  # 장애물 회피 시작 시간: 회피 동작의 타이밍 계산용
        self.st_time = None  # 주행 시작 시간: 초기 정속 주행 타이밍용

    def calc_speed(self, angle):
        """
        조향각에 따라 속도를 동적으로 계산하는 메서드
        - 입력: angle (float, 조향각, 도 단위)
        - 출력: speed (float, 계산된 속도)
        - 목적: 조향각이 클수록 속도를 줄여 안정적인 주행 유도
        """
        if angle > 0:  # 조향각이 양수 (오른쪽 회전)
            slope = -0.7  # 기울기: 오른쪽 회전 시 속도 감소
        elif angle < 0:  # 조향각이 음수 (왼쪽 회전)
            slope = 0.7  # 기울기: 왼쪽 회전 시 속도 감소
        else:  # 조향각이 0 (직진)
            slope = 0.0  # 기울기 없음: 속도 변화 없음

        speed = (slope * angle) + 30.0  # 선형 방정식: speed = slope * angle + 기본 속도(30)
        # - 예: angle=10일 때, speed = -0.7 * 10 + 30 = 23
        # - 주의: 현재 control 메서드에서 호출되지 않음 (미사용 상태)
        return speed

    def control(self):
        """
        메인 제어 루프: 장애물 회피와 차선 주행을 통합적으로 관리
        - 동작: 센서 데이터를 받아 조향각과 속도를 계산 후 모터 명령 발행
        - 주기: 10Hz로 실행 (self.rate.sleep()에 의해 보장)
        """
        try:
            # 주행 시작 후 1초간 정속 주행: 조향각 급변 방지
            if self.st_time is None:  # 주행이 처음 시작된 경우
                self.st_time = time.time()  # 시작 시간 기록
                self.motor_msg.drive.speed = 10  # 초기 속도: 느린 속도로 설정 (10 단위)
                self.motor_msg.drive.steering_angle = 0  # 초기 조향각: 직진

            # 장애물 탐지 및 회피 조향각 계산
            calc_angle = self.obstacle_detector.process(self.sensor.lidar_filtered)
            # - Clustering.process 호출: LiDAR 데이터로 장애물 회피 조향각 반환
            # - self.sensor.lidar_filtered: HPSensor에서 제공된 필터링된 LiDAR 데이터
            self.calc_angle = int(calc_angle)  # 계산된 각도를 정수로 저장

            # 장애물 회피 상태 진입
            if self.obstacle_detector.fsm.current_state in ["AvoidRight", "AvoidLeft"]:
                # - FSM 상태가 "AvoidRight" 또는 "AvoidLeft"면 장애물 감지됨
                if self.obs_time is None:  # 회피가 아직 시작되지 않은 경우
                    self.obs_time = time.time()  # 회피 시작 시간 기록
                    self.motor_msg.drive.speed = 0  # 차량 정지
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle  
                    # - 반대 방향 조향: 후진 대비 (예: 오른쪽 회피면 왼쪽으로)
                    self.obflag = True  # 장애물 회피 모드 활성화
                    print("No.0")  # 디버깅 출력: 회피 시작 알림

            # 장애물 회피 동작: 시간 기반 단계적 제어
            if self.obflag == True:  # 회피 모드가 활성화된 경우
                if time.time() - self.st_time <= 1.0:  # 주행 시작 후 1초 이내
                    self.motor_msg.drive.speed = 0  # 정지 유지
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle  # 반대 조향 유지
                    print("No.1")  # 디버깅: 초기 대기 단계
                elif time.time() - self.obs_time <= 4.0:  # 회피 시작 후 4초 이내
                    self.motor_msg.drive.steering_angle = -1.5 * self.calc_angle  
                    # - 더 강하게 반대 조향: 장애물에서 멀어짐
                    self.motor_msg.drive.speed = -15  # 후진 (음수 속도)
                    print("No.2")  # 디버깅: 후진 단계
                elif time.time() - self.obs_time <= 5.0:  # 4~5초
                    self.motor_msg.drive.steering_angle = self.calc_angle  # 원래 방향 조향
                    self.motor_msg.drive.speed = 0  # 정지
                    print("No.3")  # 디버깅: 중간 정지 단계
                elif time.time() - self.obs_time <= 6.0:  # 5~6초
                    self.motor_msg.drive.steering_angle = self.calc_angle  # 원래 방향 유지
                    self.motor_msg.drive.speed = 40  # 전진 시작
                    print("No.4")  # 디버깅: 전진 단계
                elif time.time() - self.obs_time <= 7.0:  # 6~7초
                    self.motor_msg.drive.steering_angle = 0  # 직진으로 복귀
                    self.motor_msg.drive.speed = 40  # 정상 속도 유지
                    self.obflag = False  # 회피 모드 종료
                    self.obs_time = None  # 타이머 초기화
                    print("No.5")  # 디버깅: 정상 주행 복귀

            self.motor_pub.publish(self.motor_msg)  # 모터 명령 발행: '/ackermann_cmd' 토픽으로 전송

        except ValueError:
            # LiDAR 데이터 처리 실패 시 실행: 예외 처리 블록
            # - Clustering.process에서 오류 발생 시 (예: LiDAR 데이터 없음)
            if self.obflag == True:  # 이미 회피 중인 경우
                if time.time() - self.obs_time <= 1.0:  # 0~1초
                    self.motor_msg.drive.speed = 0
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle
                    print("No.1")
                elif time.time() - self.obs_time <= 4.0:  # 1~4초
                    self.motor_msg.drive.steering_angle = -1.5 * self.calc_angle
                    self.motor_msg.drive.speed = -15
                    print("No.2")
                elif time.time() - self.obs_time <= 5.0:  # 4~5초
                    self.motor_msg.drive.steering_angle = self.calc_angle
                    self.motor_msg.drive.speed = 0
                    print("No.3")
                elif time.time() - self.obs_time <= 6.0:  # 5~6초
                    self.motor_msg.drive.steering_angle = self.calc_angle
                    self.motor_msg.drive.speed = 40
                    print("No.4")
                elif time.time() - self.obs_time <= 7.0:  # 6~7초
                    self.motor_msg.drive.steering_angle = 0
                    self.motor_msg.drive.speed = 40
                    self.obflag = False
                    self.obs_time = None
                    print("No.5")

            else:
                # 장애물이 없거나 회피 중이 아닌 경우: 차선 주행 모드
                self.obstacle_detector.fsm.transition(False)  
                # - FSM 상태를 기본 상태로 전환 (장애물 없음 신호)
                
                curvature_angle = self.lane_detector.process(self.sensor.cam)  
                # - LaneDetector.process 호출: 카메라 이미지로 차선 곡률 계산
                # - self.sensor.cam: HPSensor에서 제공된 차선 탐지용 이미지
                steering_angle = self.stanley.control(curvature_angle)  
                # - Stanley.control 호출: 차선 기반 조향각 보정

                self.motor_msg.drive.speed = int(self.speed)  # 기본 속도 (35) 적용
                self.motor_msg.drive.steering_angle = int(steering_angle)  # 조향각 설정
                print("Current motor speed: {}, Current motor angle: {}".format(
                    self.motor_msg.drive.speed, self.motor_msg.drive.steering_angle))
                # - 디버깅 출력: 현재 속도와 조향각 로그

            self.motor_pub.publish(self.motor_msg)  # 모터 명령 발행
            self.rate.sleep()  # 10Hz 주기 유지: 다음 루프까지 대기
