#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import numpy as np
import cv2

class Camera:
    def __init__(self):
        # 카메라 해상도 설정 (가로 x 세로)
        self.WIDTH, self.HEIGHT = 640, 480

        # ROI 설정: 관심 영역 (도로 부분만 보이게 설정)
        vertices = np.array([
            (0, 410), (50, 350),
            (590, 350), (640, 410)
        ], dtype=np.int32)

        # 원근 변환 전/후 좌표 정의
        self.points_src = np.float32(vertices)
        self.points_dst = np.float32([
            (100, self.HEIGHT),
            (100, 0),
            (self.WIDTH - 100, 0),
            (self.WIDTH - 100, self.HEIGHT)
        ])

        # 변환 행렬 및 역행렬 계산
        self.transform_matrix = cv2.getPerspectiveTransform(self.points_src, self.points_dst)
        self.inv_transform_matrix = cv2.getPerspectiveTransform(self.points_dst, self.points_src)

    # ========================================
    # 전처리 함수들 (실행 순서대로 정렬)
    # ========================================

    # 1. 평균 필터로 노이즈 제거
    def denoise_frame(self, img):
        kernel = np.ones((3, 3), np.float32) / 9
        return cv2.filter2D(img, -1, kernel)

    # 2. 모폴로지 열림 연산 (작은 점 제거)
    def Morphology_opening(self, img):
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        return cv2.morphologyEx(img, cv2.MORPH_OPEN, k)

    # 3. 흑백 변환
    def gray_scale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 4. 가우시안 블러 적용
    def gaussian_blur(self, img):
        return cv2.GaussianBlur(img, (3, 3), 0)

    # 5. 캐니 엣지 검출
    def canny_edges(self, img):
        return cv2.Canny(img, 80, 120)

    # 6. (선택) 캐니 엣지 재적용
    def make_canny(self, img):
        return cv2.Canny(img, 80, 120)

    # 7. Bird's Eye View 변환
    def perspective_transform(self, img):
        return cv2.warpPerspective(img, self.transform_matrix, (self.WIDTH, self.HEIGHT))

    # 부가 기능: 반사광 제거 (LAB 기반)
    def glare_removal(self, img, radius=45):
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_channel = img_lab[:, :, 0]
        l_channel = cv2.medianBlur(l_channel, radius)
        inverse_l = cv2.bitwise_not(l_channel)
        img_lab[:, :, 0] = img_lab[:, :, 0] // 2
        img_lab[:, :, 0] += inverse_l
        img_lab[:, :, 0] = img_lab[:, :, 0] // 2
        return cv2.cvtColor(img_lab, cv2.COLOR_LAB2BGR)

    # 전체 전처리 파이프라인
    def pre_processing(self, img):
        img = self.denoise_frame(img)
        img = self.Morphology_opening(img)
        img = self.gray_scale(img)
        img = self.gaussian_blur(img)
        img = self.canny_edges(img)
        img = self.make_canny(img)  # 선택적 재적용
        img = self.perspective_transform(img)
        return img
