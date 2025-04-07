#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import numpy as np
import cv2

class Camera:
    def __init__(self):
        # 카메라 해상도
        self.WIDTH, self.HEIGHT = 640, 480

        # ROI (Region of Interest) - 관심 영역 (좌하, 좌상, 우상, 우하 순서)
        vertices = np.array([
            (0, 410), (50, 350),  # 왼쪽 하단, 상단
            (590, 350), (640, 410)  # 오른쪽 상단, 하단
        ], dtype=np.int32)

        # Bird's Eye View 변환을 위한 원본(src) 및 목표(dst) 좌표 설정
        self.points_src = np.float32(vertices)
        self.points_dst = np.float32([
            (100, self.HEIGHT), (100, 0),
            (self.WIDTH - 100, 0), (self.WIDTH - 100, self.HEIGHT)
        ])

        # 투시 변환 및 역변환 행렬 생성
        self.transform_matrix = cv2.getPerspectiveTransform(self.points_src, self.points_dst)
        self.inv_transform_matrix = cv2.getPerspectiveTransform(self.points_dst, self.points_src)

    # ========================================
    # 1. 반사광 제거
    # ========================================
    def glare_removal(self, img, radius=45):
        img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_channel = img_lab[:, :, 0]
        l_channel = cv2.medianBlur(l_channel, radius)  # 급격한 밝기 변화 제거
        inverse_l = cv2.bitwise_not(l_channel)  # 밝은 부분 어둡게 (반사 제거용)
        img_lab[:, :, 0] = img_lab[:, :, 0] // 2
        img_lab[:, :, 0] += inverse_l
        img_lab[:, :, 0] = img_lab[:, :, 0] // 2
        return cv2.cvtColor(img_lab, cv2.COLOR_LAB2BGR)

    # ========================================
    # 2. 흑백 영상 변환
    # ========================================
    def gray_scale(self, img):
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # ========================================
    # 3. 노이즈 제거 (평균 필터)
    # ========================================
    def denoise_frame(self, img):
        kernel = np.ones((3, 3), np.float32) / 9
        return cv2.filter2D(img, -1, kernel)

    # ========================================
    # 4. 가우시안 블러 (노이즈 제거용)
    # ========================================
    def gaussian_blur(self, img):
        return cv2.GaussianBlur(img, (3, 3), 0)

    # ========================================
    # 5. Bird's Eye View 변환
    # ========================================
    def perspective_transform(self, img):
        return cv2.warpPerspective(img, self.transform_matrix, (self.WIDTH, self.HEIGHT))

    # ========================================
    # 6. 모폴로지 연산 (열림 연산: 침식 후 팽창)
    # ========================================
    def Morphology_opening(self, img):
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        return cv2.morphologyEx(img, cv2.MORPH_OPEN, k)

    # ========================================
    # 7. 캐니 엣지 검출 (윤곽선 추출)
    # ========================================
    def canny_edges(self, img):
        return cv2.Canny(img, 80, 120)

    # ========================================
    # 8. 캐니 엣지 재적용 (블러 등 전처리 없이 바로 캐니 적용)
    # ========================================
    def make_canny(self, img):
        return cv2.Canny(img, 80, 120)

    # ========================================
    # 9. 영상 전처리 전체 파이프라인
    # ========================================
    def pre_processing(self, img):
        # img = self.glare_removal(img)  # 반사광 제거 (옵션)
        img = self.denoise_frame(img)        # 노이즈 제거 (평균 필터)
        img = self.Morphology_opening(img)   # 모폴로지 연산 (노이즈 제거)
        img = self.gray_scale(img)           # 흑백 변환
        img = self.gaussian_blur(img)        # 가우시안 블러
        img = self.canny_edges(img)          # 엣지 검출
        img = self.make_canny(img)           # 엣지 다시 검출 (optional)
        img = self.perspective_transform(img)  # Bird's Eye View 변환
        return img
