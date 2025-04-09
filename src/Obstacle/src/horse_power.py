#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# - Python3 인터프리터를 사용하기 위한 shebang 라인: 스크립트 실행 환경 지정
# - UTF-8 인코딩 지정: 한글 등 유니코드 문자 처리를 지원

import rospy  # ROS Python 라이브러리: ROS 노드 초기화, 토픽 통신 등을 제공
from ackermann_msgs.msg import AckermannDriveStamped  # AckermannDriveStamped 메시지 타입: 속도와 조향각 데이터 구조 정의
import time  # 시간 관련 함수 모듈: 타이밍 측정 및 제어에 사용
from horse_power_sensor import HPSensor  # 사용자 정의 센서 클래스: 카메라와 LiDAR 데이터 수집 담당
from controller import Stanley  # Stanley 제어 알고리즘 클래스: 차선 주행 시 조향각을 보정
from lane_detector import LaneDetector  # 차선 탐지 클래스: 카메라 이미지에서 차선 곡률 계산
from obstacle_detector import Clustering  # 장애물 탐지 및 클러스터링 클래스: LiDAR 기반 장애물 회피 로직
from fsm import FiniteStateMachine  # 유한 상태 머신(FSM) 클래스: 장애물 회피 상태 관리

class HP:
    # HP 클래스: 자율 주행 차량의 전체 제어를 통합적으로 관리하는 핵심 클래스
    def __init__(self):
        # 객체 초기화 메서드: ROS 설정, 센서 및 제어 모듈 초기화, 초기 주행 상태 설정
        self.rate = rospy.Rate(10)  
        # - ROS 실행 주기를 10Hz(초당 10번)로 설정: 제어 루프가 일정한 속도로 동작하도록 보장
        
        self.motor_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=20)  
        # - ROS 퍼블리셔 객체 생성: '/ackermann_cmd' 토픽에 속도와 조향각 메시지를 발행
        # - queue_size=20: 최대 20개의 메시지를 대기열에 저장 가능, 데이터 손실 방지
        
        self.motor_msg = AckermannDriveStamped()  
        # - 발행할 메시지 객체 생성: 속도(drive.speed)와 조향각(drive.steering_angle)을 담을 구조체
        
        self.sensor = HPSensor()  
        # - HPSensor 객체 생성: 카메라 이미지(self.sensor.cam)와 LiDAR 데이터(self.sensor.lidar_filtered)를 수집
        
        self.sensor.init(self.rate)  
        # - 센서 초기화 호출: self.cam과 self.lidar_filtered가 None이 아닌 값으로 채워질 때까지 대기
        # - self.rate를 사용해 10Hz 주기로 대기 상태 점검
        
        self.lane_detector = LaneDetector()  
        # - LaneDetector 객체 생성: 카메라 이미지를 처리해 차선 곡률을 계산하는 모듈
        
        self.obstacle_detector = Clustering()  
        # - Clustering 객체 생성: LiDAR 데이터를 분석해 장애물을 탐지하고 회피 조향각을 계산
        
        self.stanley = Stanley()  
        # - Stanley 객체 생성: 차선 곡률을 기반으로 조향각을 보정해 차선 주행 제어

        self.speed = 35  
        # - 기본 주행 속도 설정: 차선 주행 시 사용할 정상 속도 (단위는 코드 맥락상 추정, 예: cm/s)
        
        self.calc_angle = 0  
        # - 장애물 회피용 조향각 저장 변수: Clustering.process에서 반환된 값을 유지
        
        self.obflag = False  
        # - 장애물 회피 상태 플래그: False는 비활성, True는 회피 동작 중임을 나타냄
        
        self.obs_time = None  
        # - 장애물 회피 시작 시간: 시간 기반 회피 시퀀스의 기준점으로 사용
        
        self.st_time = time.time()  
        # - 주행 시작 시간: 프로그램 초기화 시점에 현재 시간을 기록해 초기 동작 타이밍 계산에 활용
        
        self.motor_msg.drive.speed = 10  
        # - 초기 속도 설정: 시작 시 느린 속도(10)로 차량을 안정적으로 출발
        
        self.motor_msg.drive.steering_angle = 0  
        # - 초기 조향각 설정: 직진 상태(0도)로 차량 방향 초기화
        
        self.motor_pub.publish(self.motor_msg)  
        # - 초기 명령 발행: 속도 10, 조향각 0으로 설정된 메시지를 '/ackermann_cmd' 토픽에 전송

    def control(self):
        # 메인 제어 루프: 장애물 회피와 차선 주행을 통합적으로 관리하며 10Hz로 실행
        try:
            # 장애물 탐지 및 조향각 계산
            calc_angle = self.obstacle_detector.process(self.sensor.lidar_filtered)  
            # - Clustering.process 메서드 호출: LiDAR 데이터(self.sensor.lidar_filtered)를 분석
            # - 반환값: 장애물 회피를 위한 조향각(-25, 25, 0 중 하나)
            
            self.calc_angle = int(calc_angle)  
            # - 계산된 조향각을 정수로 변환해 저장: 소수점 제거로 계산 안정성 확보

            # 장애물 회피 상태 진입 체크
            if self.obstacle_detector.fsm.current_state in ["AvoidRight", "AvoidLeft"]:  
                # - FSM 상태 확인: "AvoidRight" 또는 "AvoidLeft"이면 장애물이 감지된 상태
                if self.obs_time is None:  
                    # - 회피 동작이 아직 시작되지 않은 경우 (장애물 첫 감지)
                    self.obs_time = time.time()  # 현재 시간을 기록해 회피 시작 시점 설정
                    self.motor_msg.drive.speed = 0  # 차량을 정지시켜 후진 준비
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle  
                    # - 조향각을 반대 방향으로 설정: 후진 시 장애물 반대쪽으로 이동 (예: AvoidRight면 왼쪽)
                    self.obflag = True  # 회피 모드를 활성화
                    print("No.0")  # 디버깅 출력: 회피 동작 시작을 알림

            # 장애물 회피 시퀀스: 시간 기반 7초 동작
            if self.obflag == True:  
                # - 회피 모드가 활성화된 경우: 시간 조건에 따라 단계별 동작 수행
                if time.time() - self.st_time <= 1.0:  
                    # - 주행 시작 후 0~1초: 초기 대기 단계
                    self.motor_msg.drive.speed = 0  # 차량 정지 상태 유지
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle  
                    # - 반대 방향 조향 유지: 후진 준비 완료
                    print("No.1")  # 디버깅 출력: 초기 대기 단계 확인
                
                elif time.time() - self.obs_time <= 4.0:  
                    # - 회피 시작 후 1~4초: 후진 단계
                    self.motor_msg.drive.steering_angle = -1.5 * self.calc_angle  
                    # - 반대 방향 조향을 1.5배 강화: 장애물에서 더 멀어짐
                    self.motor_msg.drive.speed = -15  # 음수 속도로 후진
                    print("No.2")  # 디버깅 출력: 후진 단계 확인
                
                elif time.time() - self.obs_time <= 5.0:  
                    # - 회피 시작 후 4~5초: 중간 정지 단계
                    self.motor_msg.drive.steering_angle = self.calc_angle  
                    # - 원래 방향으로 조향: 전진 준비
                    self.motor_msg.drive.speed = 0  # 차량 정지
                    print("No.3")  # 디버깅 출력: 중간 정지 단계 확인
                
                elif time.time() - self.obs_time <= 6.0:  
                    # - 회피 시작 후 5~6초: 전진 시작 단계
                    self.motor_msg.drive.steering_angle = self.calc_angle  
                    # - 원래 방향 유지: 장애물을 지나가는 중
                    self.motor_msg.drive.speed = 40  # 전진 속도 설정 (정상 주행보다 약간 빠름)
                    print("No.4")  # 디버깅 출력: 전진 단계 확인
                
                elif time.time() - self.obs_time <= 7.0:  
                    # - 회피 시작 후 6~7초: 정상 주행 복귀 단계
                    self.motor_msg.drive.steering_angle = 0  # 직진으로 조향 복귀
                    self.motor_msg.drive.speed = 40  # 정상 속도 유지
                    self.obflag = False  # 회피 모드 종료
                    self.obs_time = None  # 회피 타이머 초기화
                    print("No.5")  # 디버깅 출력: 정상 주행 복귀 확인

            self.motor_pub.publish(self.motor_msg)  
            # - 설정된 속도와 조향각을 '/ackermann_cmd' 토픽으로 발행: 모터에 명령 전달

        except ValueError:
            # 예외 처리: LiDAR 데이터 처리 실패 시 실행되는 로직
            # - Clustering.process에서 데이터 오류(예: msg.ranges 접근 실패)로 ValueError 발생 가능
            if self.obflag == True:  
                # - 회피 모드가 활성화된 상태라면: 이전 calc_angle 값으로 회피 시퀀스 지속
                if time.time() - self.obs_time <= 1.0:  
                    # - 회피 시작 후 0~1초: 초기 대기 단계 유지
                    self.motor_msg.drive.speed = 0  # 차량 정지
                    self.motor_msg.drive.steering_angle = -1 * self.calc_angle  # 반대 방향 조향
                    print("No.1")  # 디버깅 출력
                
                elif time.time() - self.obs_time <= 4.0:  
                    # - 회피 시작 후 1~4초: 후진 단계 지속
                    self.motor_msg.drive.steering_angle = -1.5 * self.calc_angle  # 강화된 반대 조향
                    self.motor_msg.drive.speed = -15  # 후진 속도
                    print("No.2")  # 디버깅 출력
                
                elif time.time() - self.obs_time <= 5.0:  
                    # - 회피 시작 후 4~5초: 중간 정지 단계
                    self.motor_msg.drive.steering_angle = self.calc_angle  # 원래 방향 조향
                    self.motor_msg.drive.speed = 0  # 정지
                    print("No.3")  # 디버깅 출력
                
                elif time.time() - self.obs_time <= 6.0:  
                    # - 회피 시작 후 5~6초: 전진 단계
                    self.motor_msg.drive.steering_angle = self.calc_angle  # 원래 방향 유지
                    self.motor_msg.drive.speed = 40  # 전진 속도
                    print("No.4")  # 디버깅 출력
                
                elif time.time() - self.obs_time <= 7.0:  
                    # - 회피 시작 후 6~7초: 정상 주행 복귀
                    self.motor_msg.drive.steering_angle = 0  # 직진 복귀
                    self.motor_msg.drive.speed = 40  # 정상 속도
                    self.obflag = False  # 회피 모드 종료
                    self.obs_time = None  # 타이머 초기화
                    print("No.5")  # 디버깅 출력
            
            else:
                # - 회피 모드가 비활성 상태라면: 차선 주행 모드로 전환
                self.obstacle_detector.fsm.transition(False)  
                # - FSM에 장애물 없음 신호 전달: "FollowLane" 상태로 전환
                
                curvature_angle = self.lane_detector.process(self.sensor.cam)  
                # - LaneDetector.process 호출: 카메라 이미지(self.sensor.cam)로 차선 곡률 계산
                
                steering_angle = self.stanley.control(curvature_angle)  
                # - Stanley.control 호출: 곡률 값을 기반으로 조향각을 보정
                
                self.motor_msg.drive.speed = int(self.speed)  
                # - 기본 속도(35)를 정수로 설정: 정상 주행 속도 적용
                
                self.motor_msg.drive.steering_angle = int(steering_angle)  
                # - 계산된 조향각을 정수로 설정: 차선 따라 주행
                
                print("Current motor speed: {}, Current motor angle: {}".format(
                    self.motor_msg.drive.speed, self.motor_msg.drive.steering_angle))  
                # - 디버깅 출력: 현재 속도와 조향각을 콘솔에 표시해 동작 확인

            self.motor_pub.publish(self.motor_msg)  
            # - 예외 상황에서도 설정된 속도와 조향각을 '/ackermann_cmd' 토픽으로 발행

        self.rate.sleep()  
        # - 루프 주기 유지: try와 except 실행 후 항상 10Hz(0.1초 간격)로 다음 루프 대기
