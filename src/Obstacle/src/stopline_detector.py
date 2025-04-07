#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from camera import Camera

# =============================================
# 정지선 감지를 위한 클래스 정의
# =============================================
class StoplineDetector:
    def __init__(self):
        # 카메라 객체 초기화: 이미지 데이터를 가져오기 위한 객체
        self.camera = Camera()
        # 정지선과의 거리 변수 (픽셀 단위): 초기값은 이미지 높이로 설정
        self.stopline_distance_pixel, self.prev_stopline_distance_lixel = self.camera.HEIGHT, self.camera.HEIGHT
        # 윤곽선 개수를 저장하는 리스트: 필터링에 사용
        self.contour_list = []
        # 정지선 미션 완료 여부 플래그: 0은 미완료, 이후 값으로 상태 변경 가능
        self.mission_stop_clear = 0
        # 정지선 감지 여부 플래그: 기본값은 False
        self.detected = False

    # ========================================
    # 중앙값 필터링: 윤곽선 개수의 노이즈 제거
    # 입력: 데이터(윤곽선 개수), 필터 크기(기본 5)
    # 출력: 필터링된 중앙값
    # ========================================
    def median_filter(self, data, arr_size=5):
        # 데이터가 NaN일 경우 0으로 대체: 계산 오류 방지
        if np.isnan(data):
            data = 0
        # 리스트 크기 제한: 최대 arr_size만큼 유지
        if len(self.contour_list) >= arr_size:
            self.contour_list.pop(0)  # 가장 오래된 데이터 제거
        # 새 데이터 추가
        self.contour_list.append(data)
        # 중앙값 계산: 노이즈 제거 및 안정적인 값 반환
        filtered_center = np.median(self.contour_list)
        return filtered_center

    # ========================================
    # 흰색 영역 이진화: 정지선을 강조