#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np  # 배열 연산과 수학 계산
import cv2         # 이미지 처리
from camera import Camera  # 카메라 데이터 전처리

class LaneDetector:
    # 초기화 메서드
    def __init__(self):
        self.camera = Camera()  # Camera 객체: 전처리 및 속성 제공
        self.leftx_mid, self.rightx_mid = self.camera.WIDTH * 2 // 8, self.camera.WIDTH * 6 // 8  # 초기 기준점 (160, 480)
        self.leftx_base, self.rightx_base = self.leftx_mid, self.rightx_mid  # 이전 프레임 기준점
        self.left_a, self.left_b = [0], [0]  # 왼쪽 차선 계수 (기울기, 절편)
        self.right_a, self.right_b = [0], [0]  # 오른쪽 차선 계수
        self.ploty = np.linspace(0, self.camera.HEIGHT - 1, self.camera.HEIGHT)  # y 좌표 배열 (0~479)
        self.steering_memory = 0.0  # 이전 조향각 저장

    # 슬라이딩 윈도우: 차선 위치 추적
    def sliding_window(self, img, nwindows=7, margin=30, minpix=45, draw_windows=False):
        left_fit_ = np.empty(2)  # 왼쪽 차선 계수
        right_fit_ = np.empty(2)  # 오른쪽 차선 계수
        out_img = np.dstack((img, img, img)) * 255  # 이진화 이미지를 컬러로 변환
        midpoint = self.camera.WIDTH // 2  # 중앙점 (320)
        window_height = self.camera.HEIGHT // nwindows  # 윈도우 높이 (68)
        nonzero = img.nonzero()  # 흰색 픽셀 좌표
        nonzeroy = np.array(nonzero[0])  # y 좌표 배열
        nonzerox = np.array(nonzero[1])  # x 좌표 배열
        leftx_current = self.leftx_base  # 현재 왼쪽 기준점
        rightx_current = self.rightx_base  # 현재 오른쪽 기준점
        leftx_past = leftx_current  # 이전 좌표
        rightx_past = rightx_current
        rightx_past2 = rightx_past
        left_lane_inds = []  # 왼쪽 차선 픽셀 인덱스
        right_lane_inds = []  # 오른쪽 차선 픽셀 인덱스

        # 윈도우별 차선 탐지
        for window in range(nwindows):
            if window == 0:
                win_y_low = self.camera.HEIGHT - ((window + 1) * window_height)
                win_y_high = self.camera.HEIGHT - (window * window_height)
                win_xleft_low = int(self.leftx_base - margin)
                win_xleft_high = int(self.leftx_base + margin)
                win_xright_low = int(self.rightx_base - margin)
                win_xright_high = int(self.rightx_base + margin)
            else:
                win_y_low = self.camera.HEIGHT - ((window + 1) * window_height)
                win_y_high = self.camera.HEIGHT - (window * window_height)
                win_xleft_low = int(leftx_current - margin)
                win_xleft_high = int(leftx_current + margin)
                win_xright_low = int(rightx_current - margin)
                win_xright_high = int(rightx_current + margin)

            if draw_windows:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (100, 100, 255), 3)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (100, 100, 255), 3)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            x_diff = rightx_current - leftx_current
            if len(good_left_inds) < minpix:
                if len(good_right_inds) < minpix:
                    leftx_current = leftx_current + (rightx_past - rightx_past2)
                else:
                    leftx_current = rightx_current - (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else leftx_current + (rightx_current - rightx_past)
            elif len(good_right_inds) < minpix:
                rightx_current = leftx_current + (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else rightx_current + (leftx_current - leftx_past)

            if leftx_current > midpoint - 10:
                leftx_current = midpoint - 10
            if rightx_current < midpoint + 10:
                rightx_current = midpoint + 10
            if window == 0:
                if leftx_current < 5:
                    leftx_current = 5
                if rightx_current > self.camera.WIDTH - 5:
                    rightx_current = self.camera.WIDTH - 5
                self.leftx_base = leftx_current
                self.rightx_base = rightx_current

            leftx_past = leftx_current
            rightx_past2 = rightx_past
            rightx_past = rightx_current

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        leftx = nonzerox[left_lane_inds]  # 왼쪽 차선 x 좌표
        lefty = nonzeroy[left_lane_inds]  # 왼쪽 차선 y 좌표
        rightx = nonzerox[right_lane_inds]  # 오른쪽 차선 x 좌표
        righty = nonzeroy[right_lane_inds]  # 오른쪽 차선 y 좌표

        left_lane_detected = leftx.size >= 1000
        right_lane_detected = rightx.size >= 1000

        if left_lane_detected:
            left_fit = np.polyfit(lefty, leftx, 1)
            self.left_a.append(left_fit[0])
            self.left_b.append(left_fit[1])
        if right_lane_detected:
            right_fit = np.polyfit(righty, rightx, 1)
            self.right_a.append(right_fit[0])
            self.right_b.append(right_fit[1])

        left_fit_[0] = np.mean(self.left_a[-10:])
        left_fit_[1] = np.mean(self.left_b[-10:])
        right_fit_[0] = np.mean(self.right_a[-10:])
        right_fit_[1] = np.mean(self.right_b[-10:])

        left_fitx = left_fit_[0] * self.ploty + left_fit_[1]
        right_fitx = right_fit_[0] * self.ploty + right_fit_[1]

        if not left_lane_detected and not right_lane_detected:
            self.leftx_base = self.leftx_mid
            self.rightx_base = self.rightx_mid

        return out_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected

    # 주행 경로 생성: 좌우 차선 중간 경로 계산
    def draw_path(self, img, left_fitx, right_fitx, draw_windows=False):
        left_fitx = np.array([left_fitx])
        right_fitx = np.array([right_fitx])
        path_x = np.concatenate([left_fitx, right_fitx], axis=0)
        path_x = np.mean(path_x, axis=0).reshape(-1)
        path_y = self.ploty
        if draw_windows:
            for x, y in zip(path_x, path_y):
                cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)
        return path_x, path_y

    # 조향각 계산: 경로 기반 방향 결정
    def get_angle(self, path_x, path_y, left_lane_detected, right_lane_detected):
        if not left_lane_detected and not right_lane_detected:
            return self.steering_memory * 2
        path = np.concatenate((path_x.reshape(-1, 1), path_y.reshape(-1, 1)), axis=1)
        base_diff = 320 - path[0, 0]  # 중앙(320)과의 차이
        direction = np.polyfit(path[:, 1], path[:, 0], 1)[0] * 1000  # 경로 기울기
        direction *= -0.4 if direction > 0 else -0.4  # 방향 보정
        if abs(direction) > 5:
            direction *= 0.45 + (0.05 * ((abs(direction) - 5) // 5))  # 과도한 조향 방지
        direction = max(min(direction, 20), -20)  # -20 ~ 20도 제한
        self.steering_memory = direction  # 현재 조향각 저장
        return direction

    # 전체 처리: 이미지 입력부터 조향각 출력
    def process(self, origin_img):
        origin_img = cv2.resize(origin_img, (640, 480), cv2.INTER_LINEAR)  # 크기 조정
        img = self.camera.pre_processing(origin_img)  # 이진화 처리
        sliding_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected = self.sliding_window(img, draw_windows=True)
        path_x, path_y = self.draw_path(sliding_img, left_fitx, right_fitx, draw_windows=True)
        curvature_angle_new = self.get_angle(path_x, path_y, left_lane_detected, right_lane_detected)
        return curvature_angle_new
