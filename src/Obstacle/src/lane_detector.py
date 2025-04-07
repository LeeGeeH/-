#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import numpy as np
import cv2
from camera import Camera
import math


# =============================================
# 주행을 위한 알고리즘 클래스 정의
# =============================================
class LaneDetector:

    # ========================================
    # 변수 선언 및 초기화
    # ========================================
    def __init__(self):
        # 카메라 객체 초기화
        self.camera = Camera()
        # 슬라이딩 윈도우 출력 창의 좌우 확장 값 (창 너비: WIDTH + 2*window_margin)
        self.window_margin = 20
        # 차량 크기 설정 (미터 단위)
        self.xycar_length = 0.46  # 차량 길이
        self.xycar_width = 0.3    # 차량 너비
        # 슬라이딩 윈도우 초기 기준점 (이미지 너비의 1/4과 3/4 지점)
        self.leftx_mid, self.rightx_mid = self.camera.WIDTH * 2 // 8, self.camera.WIDTH * 6 // 8
        # 이전 프레임의 기준점 초기화
        self.leftx_base, self.rightx_base = self.leftx_mid, self.rightx_mid
        # 차선 곡선 방정식 계수 리스트 (1차 방정식: ax + b)
        self.left_a, self.left_b = [0], [0]  # 왼쪽 차선 계수
        self.right_a, self.right_b = [0], [0]  # 오른쪽 차선 계수
        # 현재 차선 x 좌표 초기화 (기본값: 기준점)
        self.leftx_current, self.rightx_current = [self.leftx_mid], [self.rightx_mid]
        # 차선 간 기준 거리 및 중간점 계산
        self.ref_diff = self.rightx_mid - self.leftx_mid
        self.ref_mid = self.ref_diff // 2
        # 현재 차선 y 좌표 초기화 (이미지 하단)
        self.lefty_current, self.righty_current = [480], [480]
        # y 좌표 배열 생성 (0부터 HEIGHT-1까지 선형 분포)
        self.ploty = np.linspace(0, self.camera.HEIGHT - 1, self.camera.HEIGHT)
        # 슬라이딩 윈도우 y 좌표 (15개 구간으로 나눔)
        self.wins_y = np.linspace(464, 16, 15)
        # 경로 중앙값 및 이전 조향각 저장 변수
        self.avg_middle, self.steering_memory = 0.0, 0.0
        # 기타 변수 초기화
        self.right_aaa = 0
        self.real_angle = 0
        # 차선 인식 실패 시 부호 전환 플래그
        self.has_switched = False

    # ========================================
    # 슬라이딩 윈도우: 차선 탐지 핵심 로직
    # 입력: 이진화 이미지, 윈도우 수, 여백, 최소 픽셀, 시각화 여부
    # 출력: 결과 이미지, 차선 x 좌표, 차선 인식 여부
    # ========================================
    def sliding_window(self, img, nwindows=7, margin=30, minpix=45, draw_windows=False):
        # 차선 방정식 계수 배열 초기화 (1차 방정식용)
        left_fit_ = np.empty(2)
        right_fit_ = np.empty(2)
        # 이진화 이미지를 3채널 컬러 이미지로 변환 (시각화용)
        out_img = np.dstack((img, img, img)) * 255
        # 이미지 중앙값 계산
        midpoint = self.camera.WIDTH // 2
        # 윈도우 높이 계산 (이미지 높이를 윈도우 수로 나눔)
        window_height = self.camera.HEIGHT // nwindows
        # 이진화 이미지에서 흰색 픽셀 좌표 추출
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # 이전 기준점 가져오기
        leftx_current = self.leftx_base
        rightx_current = self.rightx_base
        # 이전 윈도우 좌표 저장 (변화량 추적용)
        leftx_past = leftx_current
        rightx_past = rightx_current
        rightx_past2 = rightx_past
        # 차선 픽셀 인덱스 저장 리스트
        left_lane_inds = []
        right_lane_inds = []
        # 슬라이딩 윈도우 중앙 좌표 저장 리스트
        left_wins_x = []
        right_wins_x = []

        # 윈도우 수만큼 반복하며 차선 탐지
        for window in range(nwindows):
            # 첫 번째 윈도우는 초기 기준점 사용, 이후는 갱신된 좌표 사용
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

            # 시각화 옵션 활성화 시 윈도우 그리기
            if draw_windows:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (100, 100, 255), 3)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (100, 100, 255), 3)

            # 윈도우 내 흰색 픽셀 인덱스 추출
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # 인덱스 저장
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # 최소 픽셀 수 이상일 경우 다음 윈도우 위치 갱신
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            # 차선 간 거리 계산
            x_diff = rightx_current - leftx_current

            # 한쪽 차선만 인식된 경우 반대쪽 추정
            if len(good_left_inds) < minpix:
                if len(good_right_inds) < minpix:
                    leftx_current = leftx_current + (rightx_past - rightx_past2)
                else:
                    leftx_current = rightx_current - (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else leftx_current + (rightx_current - rightx_past)
            elif len(good_right_inds) < minpix:
                rightx_current = leftx_current + (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else rightx_current + (leftx_current - leftx_past)

            # 기준점 제한 (중앙 넘어가지 않도록)
            if leftx_current > midpoint - 10:
                leftx_current = midpoint - 10
            if rightx_current < midpoint + 10:
                rightx_current = midpoint + 10
            # 화면 밖으로 나가지 않도록 제한 (첫 윈도우)
            if window == 0:
                if leftx_current < 5:
                    leftx_current = 5
                if rightx_current > self.camera.WIDTH - 5:
                    rightx_current = self.camera.WIDTH - 5
                # 다음 프레임 기준점 갱신
                self.leftx_base = leftx_current
                self.rightx_base = rightx_current

            # 윈도우 좌표 저장
            left_wins_x.append(leftx_current)
            right_wins_x.append(rightx_current)
            # 이전 좌표 갱신
            leftx_past = leftx_current
            rightx_past2 = rightx_past
            rightx_past = rightx_current

        # 인덱스 배열 연결
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        # 차선 픽셀 좌표 추출
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # 차선 인식 여부 판단 (픽셀 수 기준)
        left_lane_detected = leftx.size >= 1000
        right_lane_detected = rightx.size >= 1000

        # 차선 인식 시 1차 방정식 계수 계산
        if left_lane_detected:
            left_fit = np.polyfit(lefty, leftx, 1)
            self.left_a.append(left_fit[0])
            self.left_b.append(left_fit[1])
        if right_lane_detected:
            right_fit = np.polyfit(righty, rightx, 1)
            self.right_a.append(right_fit[0])
            self.right_b.append(right_fit[1])

        # 시각화: 차선 픽셀 색상 변경
        if draw_windows:
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]  # 왼쪽: 파란색
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]  # 오른쪽: 빨간색

        # 최근 10개 계수 평균으로 최종 계수 결정
        left_fit_[0] = np.mean(self.left_a[-10:])
        left_fit_[1] = np.mean(self.left_b[-10:])
        right_fit_[0] = np.mean(self.right_a[-10:])
        right_fit_[1] = np.mean(self.right_b[-10:])

        # y 값에 따른 x 좌표 계산
        left_fitx = left_fit_[0] * self.ploty + left_fit_[1]
        right_fitx = right_fit_[0] * self.ploty + right_fit_[1]

        # 차선 모두 인식 실패 시 기준점 초기화
        if not left_lane_detected and not right_lane_detected:
            self.leftx_base = self.leftx_mid
            self.rightx_base = self.rightx_mid

        return out_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected, leftx, rightx

    # ========================================
    # 주행 경로 그리기
    # 입력: 이미지, 차선 x 좌표, 시각화 여부
    # 출력: 경로 x, y 좌표
    # ========================================
    def draw_path(self, img, left_fitx, right_fitx, draw_windows=False, start_point=(320, 410)):
        # 차선 좌표를 배열로 변환
        left_fitx = np.array([left_fitx])
        right_fitx = np.array([right_fitx])
        # 경로 중앙값 계산
        path_x = np.concatenate([left_fitx, right_fitx], axis=0)
        path_x = np.mean(path_x, axis=0).reshape(-1)
        path_y = self.ploty

        # 시각화: 경로를 초록색 점으로 표시
        if draw_windows:
            for x, y in zip(path_x, path_y):
                cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)

        return path_x, path_y

    # ========================================
    # 색상 보간 함수
    # 입력: 시작 색상, 끝 색상, 보간 비율
    # 출력: 보간된 색상
    # ========================================
    def lerp_color(self, color1, color2, t):
        return tuple(map(int, color1 + t * (color2 - color1)))

    # ========================================
    # 차선 정보 반환
    # 입력: 오른쪽 차선 인식 여부
    # 출력: 차선 번호, 메시지
    # ========================================
    def lane_info(self, right_lane_detected):
        idx = 2 if right_lane_detected else 1
        lane_msg = f"lane number {idx}"
        return idx, lane_msg

    # ========================================
    # 차량 위치 및 경로 조정
    # 입력: 차선 x 좌표, 경로 좌표, 이미지, 차선 번호
    # 출력: 조정된 경로, 차이값, 상태 정보
    # ========================================
    def get_position(self, left_fitx, right_fitx, path_x, path_y, img, lane_idx):
        left_fitx = np.array([left_fitx])
        right_fitx = np.array([right_fitx])
        # 차선 기준 중간값 설정
        lane_mid = 320 if lane_idx == 1 else 330
        # 경로 중간값 계산
        mid_fitx = left_fitx + 170 if lane_idx == 1 else right_fitx - 160
        path_mid = np.mean(mid_fitx, axis=0).reshape(-1)
        base_diff = lane_mid - path_mid[-1]  # 차선 중심과의 차이

        # 경로 방향 계산
        lane_path = np.concatenate((path_mid.reshape(-1, 1), path_y.reshape(-1, 1)), axis=1)
        lane_direction = np.polyfit(lane_path[:, 1], lane_path[:, 0], 1)[0] * 1000
        # 방향 보정
        lane_direction *= -0.35 if lane_direction > 0 else -0.35
        if abs(lane_direction) > 5:
            lane_direction *= 0.4 + (0.15 if abs(lane_direction) >= 15 else 0.05 * (abs(lane_direction) // 5))
        lane_direction = max(min(lane_direction, 20), -20)

        # 경로 재생성 조건
        if abs(base_diff) > 5:
            path_x_new = np.linspace(path_mid[-1], path_mid[-1] + base_diff * 0.6 - 1, self.camera.HEIGHT)
            for x, y in zip(path_x_new, path_y):
                t = min(abs(base_diff), 50) / 50
                color = self.lerp_color(np.array([0, 255, 0]), np.array([0, 0, 255]), t)
                cv2.circle(img, (int(x), int(y)), 3, color, -1)
            path_x = path_x_new
            message = "Path Regenerate"
        else:
            message = ""

        # 상태 판단
        state, state_color = ("Stable", (0, 255, 0)) if abs(base_diff) < 10 else \
                             ("Caution", (0, 255, 255)) if abs(base_diff) < 20 else \
                             ("Alert", (0, 165, 255)) if abs(base_diff) < 30 else \
                             ("Serious", (0, 0, 255))

        return path_x, path_y, base_diff, message, state, state_color, lane_mid, lane_direction

    # ========================================
    # 점선 그리기 함수
    # 입력: 이미지, 시작/끝 점, 색상, 두께, 점선 길이
    # 출력: 점선이 그려진 이미지
    # ========================================
    def draw_dashed_line(self, img, pt1, pt2, color, thickness, dash_length):
        dist = np.linalg.norm(np.array(pt1) - np.array(pt2))
        dashes = int(dist / (2 * dash_length))
        for i in range(dashes):
            start = np.array(pt1) + i * 2 * dash_length * (np.array(pt2) - np.array(pt1)) / dist
            end = start + dash_length * (np.array(pt2) - np.array(pt1)) / dist
            cv2.line(img, tuple(map(int, start)), tuple(map(int, end)), color, thickness)
        return img

    # ========================================
    # 조향각 계산
    # 입력: 경로 좌표, 차선 인식 여부
    # 출력: 조향각
    # ========================================
    def get_angle(self, path_x, path_y, left_lane_detected, right_lane_detected):
        # 차선 모두 인식 실패 시 이전 조향각 반환
        if not left_lane_detected and not right_lane_detected:
            self.has_switched = False
            return self.steering_memory * 2
        # 차선 인식 시 조향각 계산
        self.has_switched = False
        path = np.concatenate((path_x.reshape(-1, 1), path_y.reshape(-1, 1)), axis=1)
        base_diff = 320 - path[0, 0]
        direction = np.polyfit(path[:, 1], path[:, 0], 1)[0] * 1000
        # 방향 보정
        direction *= -0.4 if direction > 0 else -0.4
        if abs(direction) > 5:
            direction *= 0.45 + (0.05 * ((abs(direction) - 5) // 5))
        direction = max(min(direction, 20), -20)
        self.steering_memory = direction
        return direction

    # ========================================
    # Main 함수: 전체 처리 로직
    # 입력: 원본 이미지
    # 출력: 조향각
    # ========================================
    def process(self, origin_img):
        # 이미지 크기 조정 및 전처리
        origin_img = cv2.resize(origin_img, (640, 480), cv2.INTER_LINEAR)
        img = self.camera.pre_processing(origin_img)
        # 슬라이딩 윈도우로 차선 탐지
        sliding_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected, leftx, rightx = self.sliding_window(img, draw_windows=True)
        # 경로 그리기
        path_x, path_y = self.draw_path(sliding_img, left_fitx, right_fitx, draw_windows=True)
        # 차선 정보 및 위치 조정
        lane_idx, lane_msg = self.lane_info(right_lane_detected)
        path_x_new, path_y_new, base_diff, message, state, state_color, lane_mid, lane_direction = self.get_position(left_fitx, right_fitx, path_x, path_y, sliding_img, lane_idx)
        # 조향각 계산
        curvature_angle_new = self.get_angle(path_x_new, path_y_new, left_lane_detected, right_lane_detected)
        # 시각화
        sliding_img = self.draw_dashed_line(sliding_img, (lane_mid, 0), (lane_mid, 480), (255, 255, 0), 2, 10)
        combined_img = self.combine_img(origin_img, self.inv_perspective_transform(sliding_img))
        # 텍스트 추가
        cv2.putText(combined_img, f'Angle: {int(curvature_angle_new)}', (0, 30), 1, 2, (255, 255, 255), 2)
        cv2.putText(combined_img, f'lane Angle: {int(lane_direction)}', (0, 60), 1, 2, (255, 255, 255), 2)
        cv2.putText(combined_img, f'base_diff: {int(base_diff)}', (0, 90), 1, 2, (255, 255, 255), 2)
        cv2.putText(combined_img, message, (320, 30), 1, 2, (255, 255, 255), 2)
        cv2.putText(combined_img, state, (400, 60), 1, 2, state_color, 2)
        cv2.putText(combined_img, "Straight" if abs(lane_direction) <= 5 else "Left" if lane_direction < -5 else "Right", (0, 120), 1, 2, (255, 255, 255), 2)
        cv2.putText(combined_img, lane_msg, (0, 150), 1, 2, (255, 255, 255), 2)
        # 이미지 표시
        cv2.imshow('Lane', combined_img)
        cv2.imshow('bird', sliding_img)
        return curvature_angle_new
