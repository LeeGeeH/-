#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# 필요한 라이브러리 임포트
import numpy as np  # 배열 연산과 수학 계산을 위한 라이브러리 (예: 차선 피팅, 평균 계산)
import cv2         # OpenCV 라이브러리로 이미지 처리 (크기 조정, 전처리, 시각화 등)
from camera import Camera  # 카메라 데이터 전처리를 위한 사용자 정의 모듈

# 차선 탐지와 조향각 계산을 위한 클래스 정의
class LaneDetector:
    # 클래스 초기화 메서드
    def __init__(self):
        # Camera 객체 생성: 영상 전처리와 기본 속성(WIDTH, HEIGHT)을 제공
        self.camera = Camera()
        # 초기 차선 기준점 설정: 이미지 너비(640)의 1/4(160)과 3/4(480) 지점으로 좌우 차선 시작점 정의
        self.leftx_mid, self.rightx_mid = self.camera.WIDTH * 2 // 8, self.camera.WIDTH * 6 // 8
        # 이전 프레임의 차선 기준점 초기화: 다음 프레임에서 슬라이딩 윈도우 시작점으로 사용
        self.leftx_base, self.rightx_base = self.leftx_mid, self.rightx_mid
        # 차선 방정식 계수 리스트 초기화: 1차 방정식(ax + b)의 기울기(a)와 절편(b)을 저장
        self.left_a, self.left_b = [0], [0]  # 왼쪽 차선 계수
        self.right_a, self.right_b = [0], [0]  # 오른쪽 차선 계수
        # 현재 차선 x 좌표 초기화: 슬라이딩 윈도우의 초기 위치로, 리스트로 관리해 변화 추적
        self.leftx_current, self.rightx_current = [self.leftx_mid], [self.rightx_mid]
        # y 좌표 배열 생성: 이미지 높이(0~479)를 선형적으로 나눠 차선 곡선 계산에 사용
        self.ploty = np.linspace(0, self.camera.HEIGHT - 1, self.camera.HEIGHT)
        # 이전 조향각 저장 변수: 차선 인식 실패 시 이전 값을 활용
        self.steering_memory = 0.0
        # 차선 인식 실패 후 부호 전환 플래그: 불필요한 상태 전환 방지
        self.has_switched = False

    # 슬라이딩 윈도우 메서드: 차선 위치를 추적하는 핵심 로직
    def sliding_window(self, img, nwindows=7, margin=30, minpix=45, draw_windows=False):
        # 차선 방정식 계수를 저장할 배열 초기화: 1차 방정식(ax + b)의 계수를 계산
        left_fit_ = np.empty(2)  # 왼쪽 차선 계수 (기울기, 절편)
        right_fit_ = np.empty(2)  # 오른쪽 차선 계수 (기울기, 절편)
        # 시각화용 컬러 이미지 생성: 이진화 이미지를 3채널(RGB)로 변환, 흰색 픽셀 강조
        out_img = np.dstack((img, img, img)) * 255
        # 이미지 중앙값 계산: 차선이 중앙을 넘어가지 않도록 제한할 기준
        midpoint = self.camera.WIDTH // 2  # 640 // 2 = 320
        # 윈도우 높이 계산: 이미지 높이(480)를 윈도우 수(7)로 나눠 각 윈도우의 세로 크기 결정
        window_height = self.camera.HEIGHT // nwindows  # 480 // 7 ≈ 68
        # 이진화 이미지에서 흰색 픽셀 좌표 추출: 차선 위치를 찾기 위한 기본 데이터
        nonzero = img.nonzero()  # (y 좌표 배열, x 좌표 배열) 반환
        nonzeroy = np.array(nonzero[0])  # 모든 흰색 픽셀의 y 좌표
        nonzerox = np.array(nonzero[1])  # 모든 흰색 픽셀의 x 좌표
        # 현재 윈도우의 차선 중심 초기화: 이전 프레임의 기준점에서 시작
        leftx_current = self.leftx_base
        rightx_current = self.rightx_base
        # 이전 윈도우 좌표 저장: 차선 변화량 추적 및 추정에 사용
        leftx_past = leftx_current
        rightx_past = rightx_current
        rightx_past2 = rightx_past
        # 차선 픽셀 인덱스를 저장할 리스트: 각 윈도우에서 탐지된 픽셀 모음
        left_lane_inds = []
        right_lane_inds = []

        # 윈도우 수만큼 반복: 이미지 하단에서 상단으로 차선 탐지
        for window in range(nwindows):
            # 윈도우의 y 범위 설정: 하단(480)에서 상단(0)으로 올라가며 구간 나눔
            if window == 0:
                # 첫 번째 윈도우: 초기 기준점 사용
                win_y_low = self.camera.HEIGHT - ((window + 1) * window_height)  # 예: 480 - 68 = 412
                win_y_high = self.camera.HEIGHT - (window * window_height)  # 예: 480 - 0 = 480
                win_xleft_low = int(self.leftx_base - margin)  # 예: 160 - 30 = 130
                win_xleft_high = int(self.leftx_base + margin)  # 예: 160 + 30 = 190
                win_xright_low = int(self.rightx_base - margin)  # 예: 480 - 30 = 450
                win_xright_high = int(self.rightx_base + margin)  # 예: 480 + 30 = 510
            else:
                # 이후 윈도우: 갱신된 좌표 사용
                win_y_low = self.camera.HEIGHT - ((window + 1) * window_height)
                win_y_high = self.camera.HEIGHT - (window * window_height)
                win_xleft_low = int(leftx_current - margin)
                win_xleft_high = int(leftx_current + margin)
                win_xright_low = int(rightx_current - margin)
                win_xright_high = int(rightx_current + margin)

            # 시각화 옵션: 윈도우를 이미지에 그리기 (디버깅용)
            if draw_windows:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (100, 100, 255), 3)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (100, 100, 255), 3)

            # 윈도우 내 흰색 픽셀 인덱스 추출: 조건에 맞는 픽셀만 선택
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # 탐지된 픽셀 인덱스 저장: 나중에 차선 좌표 추출에 사용
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # 픽셀이 최소 개수(minpix) 이상이면 중심 갱신
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))  # 평균 x 좌표로 갱신
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            # 차선 간 거리 계산: 한쪽만 인식된 경우 반대쪽 추정에 사용
            x_diff = rightx_current - leftx_current
            if len(good_left_inds) < minpix:
                if len(good_right_inds) < minpix:
                    # 양쪽 모두 실패: 이전 변화량으로 추정
                    leftx_current = leftx_current + (rightx_past - rightx_past2)
                else:
                    # 오른쪽만 인식: 왼쪽을 오른쪽 기준으로 추정
                    leftx_current = rightx_current - (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else leftx_current + (rightx_current - rightx_past)
            elif len(good_right_inds) < minpix:
                # 왼쪽만 인식: 오른쪽을 왼쪽 기준으로 추정
                rightx_current = leftx_current + (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else rightx_current + (leftx_current - leftx_past)

            # 차선 위치 제한: 중앙 넘어가지 않도록 조정
            if leftx_current > midpoint - 10:
                leftx_current = midpoint - 10  # 왼쪽 차선이 중앙(320) 근처로 제한
            if rightx_current < midpoint + 10:
                rightx_current = midpoint + 10  # 오른쪽 차선도 중앙 근처로 제한
            if window == 0:
                # 첫 윈도우: 화면 밖으로 나가지 않도록 제한 및 기준점 갱신
                if leftx_current < 5:
                    leftx_current = 5
                if rightx_current > self.camera.WIDTH - 5:
                    rightx_current = self.camera.WIDTH - 5
                self.leftx_base = leftx_current  # 다음 프레임 시작점으로 저장
                self.rightx_base = rightx_current

            # 이전 좌표 갱신: 변화량 계산에 사용
            leftx_past = leftx_current
            rightx_past2 = rightx_past
            rightx_past = rightx_current

        # 모든 윈도우의 인덱스 연결: 차선 픽셀 좌표를 하나로 모음
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        leftx = nonzerox[left_lane_inds]  # 왼쪽 차선 x 좌표
        lefty = nonzeroy[left_lane_inds]  # 왼쪽 차선 y 좌표
        rightx = nonzerox[right_lane_inds]  # 오른쪽 차선 x 좌표
        righty = nonzeroy[right_lane_inds]  # 오른쪽 차선 y 좌표

        # 차선 인식 여부 판단: 픽셀 수가 1000 이상이면 인식 성공
        left_lane_detected = leftx.size >= 1000
        right_lane_detected = rightx.size >= 1000

        # 차선 인식 성공 시 1차 방정식 피팅: ax + b 형태로 계산
        if left_lane_detected:
            left_fit = np.polyfit(lefty, leftx, 1)  # [기울기, 절편]
            self.left_a.append(left_fit[0])  # 기울기 저장
            self.left_b.append(left_fit[1])  # 절편 저장
        if right_lane_detected:
            right_fit = np.polyfit(righty, rightx, 1)
            self.right_a.append(right_fit[0])
            self.right_b.append(right_fit[1])

        # 최근 10프레임 계수 평균: 부드러운 차선 추적을 위해 사용
        left_fit_[0] = np.mean(self.left_a[-10:])  # 왼쪽 기울기 평균
        left_fit_[1] = np.mean(self.left_b[-10:])  # 왼쪽 절편 평균
        right_fit_[0] = np.mean(self.right_a[-10:])  # 오른쪽 기울기 평균
        right_fit_[1] = np.mean(self.right_b[-10:])  # 오른쪽 절편 평균

        # y 값에 따른 x 좌표 계산: 차선 곡선을 전체 높이에 맞춰 생성
        left_fitx = left_fit_[0] * self.ploty + left_fit_[1]  # 왼쪽 차선 x 좌표
        right_fitx = right_fit_[0] * self.ploty + right_fit_[1]  # 오른쪽 차선 x 좌표

        # 차선 모두 인식 실패 시 기준점 초기화: 다음 프레임에서 기본값으로 복귀
        if not left_lane_detected and not right_lane_detected:
            self.leftx_base = self.leftx_mid
            self.rightx_base = self.rightx_mid

        # 결과 반환: 시각화 이미지, 차선 좌표, 인식 여부
        return out_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected, leftx, rightx

    # 주행 경로 생성 메서드: 좌우 차선 중간에 경로 계산
    def draw_path(self, img, left_fitx, right_fitx, draw_windows=False):
        # 차선 좌표를 배열로 변환: 계산 편의를 위해 2D 배열로
        left_fitx = np.array([left_fitx])
        right_fitx = np.array([right_fitx])
        # 좌우 차선 결합 후 평균 계산: 경로의 x 좌표 생성
        path_x = np.concatenate([left_fitx, right_fitx], axis=0)
        path_x = np.mean(path_x, axis=0).reshape(-1)  # 1D 배열로 변환
        path_y = self.ploty  # y 좌표는 이미 정의된 값 사용
        # 시각화 옵션: 경로를 초록색 점으로 표시 (디버깅용)
        if draw_windows:
            for x, y in zip(path_x, path_y):
                cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)  # 반지름 3, 초록색, 채우기
        # 결과 반환: 경로 x, y 좌표
        return path_x, path_y

    # 조향각 계산 메서드: 경로 기반으로 차량 방향 결정
    def get_angle(self, path_x, path_y, left_lane_detected, right_lane_detected):
        # 차선 모두 인식 실패: 이전 조향각의 2배 반환
        if not left_lane_detected and not right_lane_detected:
            self.has_switched = False  # 상태 플래그 초기화
            return self.steering_memory * 2  # 이전 값 활용으로 연속성 유지
        # 차선 인식 성공 시 조향각 계산
        self.has_switched = False  # 상태 플래그 초기화
        # 경로를 2D 배열로 결합: x, y 좌표 쌍으로 변환
        path = np.concatenate((path_x.reshape(-1, 1), path_y.reshape(-1, 1)), axis=1)
        # 이미지 하단에서 차량과 경로의 차이 계산: 중앙(320) 기준
        base_diff = 320 - path[0, 0]
        # 경로 방향 계산: 1차 방정식 기울기를 구하고 1000배로 확대
        direction = np.polyfit(path[:, 1], path[:, 0], 1)[0] * 1000
        # 방향 보정: 음수/양수에 따라 반전 및 크기 조정
        direction *= -0.4 if direction > 0 else -0.4
        # 방향 값이 클 경우 추가 보정: 과도한 조향 방지
        if abs(direction) > 5:
            direction *= 0.45 + (0.05 * ((abs(direction) - 5) // 5))
        # 조향각 제한: -20 ~ 20도 사이로 유지
        direction = max(min(direction, 20), -20)
        # 현재 조향각 저장: 다음 프레임에서 실패 시 사용
        self.steering_memory = direction
        # 결과 반환: 계산된 조향각
        return direction

    # 전체 처리 메서드: 이미지 입력부터 조향각 출력까지
    def process(self, origin_img):
        # 이미지 크기 조정: 입력 이미지를 640x480으로 변환 (일관성 유지)
        origin_img = cv2.resize(origin_img, (640, 480), cv2.INTER_LINEAR)
        # 전처리: Camera 모듈로 이진화 이미지 생성 (차선 강조)
        img = self.camera.pre_processing(origin_img)
        # 슬라이딩 윈도우 실행: 차선 탐지 및 좌표 계산
        sliding_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected, leftx, rightx = self.sliding_window(img, draw_windows=True)
        # 경로 생성: 좌우 차선 중간에 주행 경로 계산
        path_x, path_y = self.draw_path(sliding_img, left_fitx, right_fitx, draw_windows=True)
        # 조향각 계산: 경로 기반으로 방향 결정
        curvature_angle_new = self.get_angle(path_x, path_y, left_lane_detected, right_lane_detected)
        # 결과 반환: 계산된 조향각
        return curvature_angle_new
