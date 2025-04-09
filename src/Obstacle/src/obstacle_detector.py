#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import math
from sklearn.cluster import DBSCAN
import os
######################
import rospy
from time import time
from ackermann_msgs.msg import AckermannDriveStamped
from FSM import FiniteStateMachine
######################

# =============================================
# 장애물 감지 및 회피를 위한 클러스터링 클래스 정의
# =============================================
class Clustering:
    def __init__(self):
        # ROS 관련 초기화 ##################################################
        # ROS 노드 실행 주기 설정 (10Hz)
        self.rate = rospy.Rate(10)
        # 조향각과 속도를 퍼블리시할 토픽 설정
        self.motor_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=20)
        # 모터 제어 메시지 객체 생성
        self.motor_msg = AckermannDriveStamped()
        # 유한 상태 머신(FSM) 객체 초기화: 상태 관리용
        self.fsm = FiniteStateMachine()
        ###################################################################

        # 클러스터링 관련 설정 ############################################
        # DBSCAN 파라미터: 반경 1m 내 최소 30개 포인트로 클러스터 판단
        epsilon = 1.0  # 클러스터 중심점 기준 반경 (미터)
        min_sample = 30  # 최소 포인트 수 (이보다 적으면 노이즈로 간주)
        # DBSCAN 모델 초기화: ball_tree 알고리즘으로 효율적 클러스터링
        self.model = DBSCAN(eps=epsilon, min_samples=min_sample, algorithm='ball_tree', leaf_size=20)
        ###################################################################

        # LiDAR 측정 범위 설정 ############################################
        # 탐지 범위: 1.4m 이내, 각도 ±30도 (최적값으로 설정)
        self.limit_range = 1.4  # 거리 제한 (미터)
        self.limit_degree = 30  # 각도 제한 (도)
        ###################################################################

        # 제어 관련 변수 초기화 ###########################################
        self.steering_angle = 0  # 조향각 초기값
        self.count = 0  # 카운터 (미션 상태 추적용 추정)
        self.wait_flag = 0  # 대기 플래그 (미션 대기용 추정)
        self.BOTH, self.LEFT, self.RIGHT = 0, 1, 2  # ROI 설정 상수
        self.roi_setting = self.BOTH  # 기본 ROI: 양쪽 탐지
        self.mission_finished = False  # 미션 완료 여부 플래그
        ###################################################################

    # ========================================
    # 좌표 변환: LiDAR 데이터를 x, y 좌표로 변환
    # 입력: LiDAR 메시지 (msg)
    # 출력: 변환된 좌표 배열
    # ========================================
    def coordinate_transform(self, msg):
        # LiDAR 각도 배열 생성: 각 인덱스에 해당하는 각도 계산
        # angle = angle_min + angle_increment * index (라디안 -> 도 변환)
        degree = [(msg.angle_min + msg.angle_increment * idx) * (180 / math.pi) for idx, value in enumerate(msg.ranges)]
        ranges = msg.ranges  # LiDAR 거리 데이터

        # 관심 영역(ROI) 내 데이터만 추출하기 위한 배열 초기화
        ranged_degree = np.array([])
        ranged_ranges = np.array([])

        # 원하는 각도 범위 설정: 정면(90도)을 기준으로 ±limit_degree
        desired_center_angle = 90
        desired_min_angle = desired_center_angle - self.limit_degree  # 예: 60도
        desired_max_angle = desired_center_angle + self.limit_degree  # 예: 120도

        # ROI 내 데이터 필터링
        for idx, value in enumerate(msg.ranges):
            # 조건: 각도가 ROI 내이고 거리가 limit_range 이내
            if (desired_min_angle <= degree[idx] <= desired_max_angle) and ranges[idx] <= self.limit_range:
                # 무한대 값 처리: NaN으로 변환
                if ranges[idx] is np.inf:
                    ranges[idx] = np.NaN
                # 필터링된 각도와 거리 저장
                ranged_degree = np.append(ranged_degree, degree[idx])
                ranged_ranges = np.append(ranged_ranges, ranges[idx])

        # 각도를 라디안으로 변환
        thetas = np.deg2rad(ranged_degree)
        # 직교 좌표계로 변환: x = r*cos(θ), y = r*sin(θ)
        x = ranged_ranges * np.cos(thetas)
        y = ranged_ranges * np.sin(thetas)
        # x, y 좌표를 2D 배열로 결합
        coordinate = np.column_stack((x, y))

        return coordinate

    # ========================================
    # 클러스터링: LiDAR 데이터를 군집화
    # 입력: 변환된 좌표 데이터
    # 출력: 클러스터 중심점 리스트
    # ========================================
    def clustering(self, data):
        # 클러스터 중심점을 저장할 리스트
        centroid_list = []
        # DBSCAN으로 데이터 학습: 군집 생성
        self.model.fit(data)

        # 각 고유 레이블에 대해 처리
        for label in list(np.unique(self.model.labels_)):
            if label == -1:  # -1은 노이즈로 판단
                continue
            # 해당 레이블의 데이터 추출
            label_index = np.where(self.model.labels_ == label)
            cluster = data[label_index]
            # 클러스터의 중심점 계산 (평균)
            centroid = np.mean(cluster, axis=0)
            centroid_list.append(centroid)

        print("Detect : ", np.unique(self.model.labels_))
        return centroid_list

    # ========================================
    # 장애물 회피: 조향각 계산
    # 입력: 클러스터 중심점 리스트
    # 출력: 조향각 (도)
    # ========================================
    def avoidance(self, centroid_list):
        # 중심점과의 유클리드 거리 계산
        distance_list = [np.linalg.norm(centroid) for centroid in centroid_list]
        # 가장 가까운 장애물의 중심점 선택 (x, y 순서 역으로 적용)
        closest_centroid = centroid_list[np.argmin(distance_list)][::-1] if distance_list else None

        # 장애물 감지 여부 확인
        detected_obstacle = bool(centroid_list)
        print("detected_obstacle {}".format(detected_obstacle))

        # FSM으로 상태 전이 수행
        self.fsm.transition(detected_obstacle)

        # 상태에 따른 조향각 설정
        if self.fsm.current_state == "AvoidLeft":
            angle = -25  # 왼쪽 회피
        elif self.fsm.current_state == "AvoidRight":
            angle = 25  # 오른쪽 회피
        else:
            angle = 0  # 기본값 (직진 추정)

        # 조향각 범위 제한: -25도 ~ 25도
        angle = max(min(angle, 25.0), -25.0)

        return angle

    # ========================================
    # 메인 처리 함수: LiDAR 데이터 처리 및 조향각 반환
    # 입력: LiDAR 메시지 (msg)
    # 출력: 조향각
    # ========================================
    def process(self, msg):
        # LiDAR 데이터를 좌표로 변환
        coordinate = self.coordinate_transform(msg)
        # 클러스터링 수행
        centroid_list = self.clustering(coordinate)
        # 회피 조향각 계산
        self.steering_angle = self.avoidance(centroid_list)
        print("centroid_list : {}".format(centroid_list))
        return self.steering_angle
