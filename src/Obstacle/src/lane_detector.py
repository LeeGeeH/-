#!/usr/bin/env python3
# -*- coding:utf-8 -*-
# - Python3 실행을 위한 shebang 라인
# - UTF-8 인코딩으로 유니코드 문자 지원 (예: 한글 주석 가능)

import numpy as np  # 배열 연산과 수학 계산을 위한 라이브러리 (예: 차선 피팅, 평균 계산, 좌표 변환)
import cv2         # OpenCV 라이브러리로 이미지 처리 (크기 조정, 이진화, 시각화 등)
from camera import Camera  # 사용자 정의 모듈로 카메라 이미지 전처리 제공 (예: 이진화, 노이즈 제거)

class LaneDetector:
    # 차선 탐지와 조향각 계산을 위한 클래스
    def __init__(self):
        # 클래스 초기화 메서드: 객체 생성 시 기본 설정 정의
        self.camera = Camera()  # Camera 객체 생성: 이미지 전처리와 속성(WIDTH=640, HEIGHT=480) 제공
        # 초기 차선 기준점 설정: 이미지 너비(640)를 8등분해 2/8(160)과 6/8(480) 지점을 좌우 차선 시작점으로 정의
        self.leftx_mid, self.rightx_mid = self.camera.WIDTH * 2 // 8, self.camera.WIDTH * 6 // 8
        # 이전 프레임의 차선 기준점 초기화: 다음 프레임에서 슬라이딩 윈도우의 시작점으로 사용
        self.leftx_base, self.rightx_base = self.leftx_mid, self.rightx_mid
        # 차선 방정식 계수 리스트 초기화: 1차 방정식(ax + b)의 기울기(a)와 절편(b)을 저장
        self.left_a, self.left_b = [0], [0]  # 왼쪽 차선의 기울기와 절편 리스트, 초기값 0
        self.right_a, self.right_b = [0], [0]  # 오른쪽 차선의 기울기와 절편 리스트, 초기값 0
        # y 좌표 배열 생성: 이미지 높이(480)를 0부터 479까지 480개 점으로 선형 분할
        # - 용도: 차선 곡선 계산 시 y 값을 고정하고 x 값을 구함
        self.ploty = np.linspace(0, self.camera.HEIGHT - 1, self.camera.HEIGHT)
        # 이전 조향각 저장 변수: 차선 인식 실패 시 이전 값을 활용해 연속성 유지
        self.steering_memory = 0.0

    # 슬라이딩 윈도우 메서드: 이미지에서 차선 위치를 추적하는 핵심 로직
    def sliding_window(self, img, nwindows=7, margin=30, minpix=45, draw_windows=False):
        # 입력: 이진화 이미지(img), 윈도우 수(nwindows), 윈도우 너비(margin), 최소 픽셀 수(minpix), 시각화 여부(draw_windows)
        # 차선 방정식 계수를 저장할 배열 초기화: 1차 방정식(ax + b)의 계수 저장용
        left_fit_ = np.empty(2)  # 왼쪽 차선 계수 배열: [기울기, 절편]
        right_fit_ = np.empty(2)  # 오른쪽 차선 계수 배열: [기울기, 절편]
        # 시각화용 컬러 이미지 생성: 이진화 이미지(0/1)를 3채널(RGB)로 변환
        # - np.dstack: (480, 640) 배열 3개를 쌓아 (480, 640, 3)으로 만듦
        # - * 255: 0/1 값을 0/255로 확장해 흰색 픽셀 강조
        out_img = np.dstack((img, img, img)) * 255
        # 이미지 중앙값 계산: 차선이 중앙을 넘어가지 않도록 제한할 기준점
        midpoint = self.camera.WIDTH // 2  # 640 // 2 = 320
        # 윈도우 높이 계산: 이미지 높이(480)를 윈도우 수(7)로 나눠 각 윈도우의 세로 크기 결정
        window_height = self.camera.HEIGHT // nwindows  # 480 // 7 ≈ 68
        # 이진화 이미지에서 흰색 픽셀 좌표 추출: 차선 위치를 찾기 위한 기본 데이터
        # - nonzero(): (y 좌표 배열, x 좌표 배열) 튜플 반환
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])  # 모든 흰색 픽셀의 y 좌표 (예: [0, 1, 5, ...])
        nonzerox = np.array(nonzero[1])  # 모든 흰색 픽셀의 x 좌표 (예: [100, 150, 200, ...])
        # 현재 윈도우의 차선 중심 초기화: 이전 프레임의 기준점에서 시작
        leftx_current = self.leftx_base  # 왼쪽 차선 시작점 (초기값 160)
        rightx_current = self.rightx_base  # 오른쪽 차선 시작점 (초기값 480)
        # 이전 윈도우 좌표 저장: 차선 변화량 추적 및 추정에 사용
        leftx_past = leftx_current  # 직전 윈도우의 왼쪽 x 좌표
        rightx_past = rightx_current  # 직전 윈도우의 오른쪽 x 좌표
        rightx_past2 = rightx_past  # 두 번 전 윈도우의 오른쪽 x 좌표 (변화량 계산용)
        # 차선 픽셀 인덱스를 저장할 리스트: 각 윈도우에서 탐지된 픽셀의 인덱스 모음
        left_lane_inds = []  # 왼쪽 차선 픽셀 인덱스 리스트
        right_lane_inds = []  # 오른쪽 차선 픽셀 인덱스 리스트

        # 윈도우 수(7)만큼 반복: 이미지 하단에서 상단으로 차선 탐지
        for window in range(nwindows):
            # 윈도우의 y 범위 설정: 하단(480)에서 상단(0)으로 올라가며 구간 나눔
            if window == 0:
                # 첫 번째 윈도우: 초기 기준점 사용
                win_y_low = self.camera.HEIGHT - ((window + 1) * window_height)  # 480 - 68 = 412
                win_y_high = self.camera.HEIGHT - (window * window_height)  # 480 - 0 = 480
                win_xleft_low = int(self.leftx_base - margin)  # 160 - 30 = 130 (왼쪽 경계)
                win_xleft_high = int(self.leftx_base + margin)  # 160 + 30 = 190 (오른쪽 경계)
                win_xright_low = int(self.rightx_base - margin)  # 480 - 30 = 450
                win_xright_high = int(self.rightx_base + margin)  # 480 + 30 = 510
            else:
                # 이후 윈도우: 이전 윈도우에서 갱신된 좌표 사용
                win_y_low = self.camera.HEIGHT - ((window + 1) * window_height)  # 예: 480 - 136 = 344
                win_y_high = self.camera.HEIGHT - (window * window_height)  # 예: 480 - 68 = 412
                win_xleft_low = int(leftx_current - margin)  # 현재 중심 - 30
                win_xleft_high = int(leftx_current + margin)  # 현재 중심 + 30
                win_xright_low = int(rightx_current - margin)
                win_xright_high = int(rightx_current + margin)

            # 시각화 옵션: 윈도우를 이미지에 그리기 (디버깅용)
            if draw_windows:
                # cv2.rectangle: 사각형 그리기 (시작점, 끝점, 색상(BGR), 두께)
                # - 왼쪽 윈도우: 빨간색(100, 100, 255), 두께 3px
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (100, 100, 255), 3)
                # - 오른쪽 윈도우: 동일한 스타일로 그리기
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (100, 100, 255), 3)

            # 윈도우 내 흰색 픽셀 인덱스 추출: 조건에 맞는 픽셀만 선택
            # - 논리 연산(&)으로 y, x 범위 내 픽셀 필터링 후 인덱스 반환
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # 탐지된 픽셀 인덱스 저장: 나중에 차선 좌표 추출에 사용
            left_lane_inds.append(good_left_inds)  # 왼쪽 윈도우 인덱스 추가
            right_lane_inds.append(good_right_inds)  # 오른쪽 윈도우 인덱스 추가

            # 픽셀이 최소 개수(minpix=45) 이상이면 중심 갱신
            if len(good_left_inds) > minpix:
                # 평균 x 좌표로 갱신: 탐지된 픽셀의 x 값 평균 계산
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            # 차선 간 거리 계산: 한쪽만 인식된 경우 반대쪽 추정에 사용
            x_diff = rightx_current - leftx_current  # 오른쪽 - 왼쪽 거리
            if len(good_left_inds) < minpix:
                if len(good_right_inds) < minpix:
                    # 양쪽 모두 실패: 이전 오른쪽 변화량으로 왼쪽 추정
                    # - rightx_past - rightx_past2: 직전 두 윈도우의 이동량
                    leftx_current = leftx_current + (rightx_past - rightx_past2)
                else:
                    # 오른쪽만 인식: 왼쪽을 오른쪽 기준으로 추정
                    # - x_diff < 320: 정상적인 차선 간격이면 중앙(320) 차이로 설정
                    # - 그렇지 않으면 오른쪽 변화량으로 보정
                    leftx_current = rightx_current - (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else leftx_current + (rightx_current - rightx_past)
            elif len(good_right_inds) < minpix:
                # 왼쪽만 인식: 오른쪽을 왼쪽 기준으로 추정
                rightx_current = leftx_current + (self.camera.WIDTH // 2) if x_diff < self.camera.WIDTH // 2 else rightx_current + (leftx_current - leftx_past)

            # 차선 위치 제한: 중앙 넘어가지 않도록 조정
            if leftx_current > midpoint - 10:
                leftx_current = midpoint - 10  # 왼쪽 차선 최대 310으로 제한
            if rightx_current < midpoint + 10:
                rightx_current = midpoint + 10  # 오른쪽 차선 최소 330으로 제한
            if window == 0:
                # 첫 윈도우: 화면 밖으로 나가지 않도록 제한 및 기준점 갱신
                if leftx_current < 5:
                    leftx_current = 5  # 최소 5px
                if rightx_current > self.camera.WIDTH - 5:
                    rightx_current = self.camera.WIDTH - 5  # 최대 635px
                self.leftx_base = leftx_current  # 다음 프레임 시작점으로 저장
                self.rightx_base = rightx_current

            # 이전 좌표 갱신: 다음 윈도우에서 변화량 계산에 사용
            leftx_past = leftx_current
            rightx_past2 = rightx_past  # 두 번 전 값으로 이동
            rightx_past = rightx_current  # 직전 값으로 갱신

        # 모든 윈도우의 인덱스 연결: 리스트를 단일 배열로 결합
        left_lane_inds = np.concatenate(left_lane_inds)  # 왼쪽 차선 전체 인덱스
        right_lane_inds = np.concatenate(right_lane_inds)  # 오른쪽 차선 전체 인덱스
        # 차선 픽셀 좌표 추출: 인덱스를 이용해 x, y 좌표 모음
        leftx = nonzerox[left_lane_inds]  # 왼쪽 차선 x 좌표 배열
        lefty = nonzeroy[left_lane_inds]  # 왼쪽 차선 y 좌표 배열
        rightx = nonzerox[right_lane_inds]  # 오른쪽 차선 x 좌표 배열
        righty = nonzeroy[right_lane_inds]  # 오른쪽 차선 y 좌표 배열

        # 차선 인식 여부 판단: 픽셀 수가 1000 이상이면 인식 성공으로 간주
        left_lane_detected = leftx.size >= 1000
        right_lane_detected = rightx.size >= 1000

        # 차선 인식 성공 시 1차 방정식 피팅: ax + b 형태로 계산
        if left_lane_detected:
            # np.polyfit: y에 대한 x의 1차 다항식 계수 반환 ([기울기, 절편])
            left_fit = np.polyfit(lefty, leftx, 1)
            self.left_a.append(left_fit[0])  # 기울기 저장
            self.left_b.append(left_fit[1])  # 절편 저장
        if right_lane_detected:
            right_fit = np.polyfit(righty, rightx, 1)
            self.right_a.append(right_fit[0])
            self.right_b.append(right_fit[1])

        # 최근 10프레임 계수 평균: 부드러운 차선 추적을 위해 사용
        # - [-10:]: 리스트의 마지막 10개 요소 슬라이싱, 초기값 포함
        left_fit_[0] = np.mean(self.left_a[-10:])  # 왼쪽 기울기 평균
        left_fit_[1] = np.mean(self.left_b[-10:])  # 왼쪽 절편 평균
        right_fit_[0] = np.mean(self.right_a[-10:])  # 오른쪽 기울기 평균
        right_fit_[1] = np.mean(self.right_b[-10:])  # 오른쪽 절편 평균

        # y 값에 따른 x 좌표 계산: 차선 곡선을 전체 높이(0~479)에 맞춰 생성
        # - 방정식: x = a * y + b
        left_fitx = left_fit_[0] * self.ploty + left_fit_[1]  # 왼쪽 차선 x 좌표 (480개)
        right_fitx = right_fit_[0] * self.ploty + right_fit_[1]  # 오른쪽 차선 x 좌표 (480개)

        # 차선 모두 인식 실패 시 기준점 초기화: 다음 프레임에서 기본값(160, 480)으로 복귀
        if not left_lane_detected and not right_lane_detected:
            self.leftx_base = self.leftx_mid
            self.rightx_base = self.rightx_mid

        # 결과 반환: 시각화 이미지, 차선 좌표, 인식 여부
        return out_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected

    # 주행 경로 생성 메서드: 좌우 차선 중간에 경로 계산
    def draw_path(self, img, left_fitx, right_fitx, draw_windows=False):
        # 입력: 시각화 이미지(img), 좌우 차선 x 좌표(left_fitx, right_fitx), 시각화 여부(draw_windows)
        # 차선 좌표를 배열로 변환: 계산 편의를 위해 2D 배열로 변환
        # - [left_fitx]: (480,) → (1, 480)
        left_fitx = np.array([left_fitx])
        right_fitx = np.array([right_fitx])
        # 좌우 차선 결합: (2, 480) 배열 생성
        # - axis=0: 행 방향으로 쌓기
        path_x = np.concatenate([left_fitx, right_fitx], axis=0)
        # 평균 계산: 각 y 값에 대해 좌우 x 좌표 평균 내기
        # - (2, 480) → (480,)로 축소
        path_x = np.mean(path_x, axis=0).reshape(-1)
        path_y = self.ploty  # y 좌표는 고정값(0~479) 사용
        # 시각화 옵션: 경로를 초록색 점으로 표시 (디버깅용)
        if draw_windows:
            # zip: path_x와 path_y를 쌍으로 묶어 순회
            for x, y in zip(path_x, path_y):
                # cv2.circle: 원 그리기 (중심점, 반지름, 색상(BGR), 두께(-1은 채우기))
                # - 초록색(0, 255, 0), 반지름 3px, 채운 원
                cv2.circle(img, (int(x), int(y)), 3, (0, 255, 0), -1)
        # 결과 반환: 경로 x, y 좌표
        return path_x, path_y

    # 조향각 계산 메서드: 경로 기반으로 차량 방향 결정
    def get_angle(self, path_x, path_y, left_lane_detected, right_lane_detected):
        # 입력: 경로 x, y 좌표, 좌우 차선 인식 여부
        # 차선 모두 인식 실패: 이전 조향각의 2배 반환
        if not left_lane_detected and not right_lane_detected:
            # - 2배로 반환: 차선이 없으면 더 강하게 반응하도록 설계
            return self.steering_memory * 2
        # 경로를 2D 배열로 결합: x, y 좌표 쌍으로 변환
        # - reshape(-1, 1): (480,) → (480, 1)
        # - axis=1: 열 방향으로 결합 → (480, 2)
        path = np.concatenate((path_x.reshape(-1, 1), path_y.reshape(-1, 1)), axis=1)
        # 이미지 하단에서 차량과 경로의 차이 계산: 중앙(320) 기준
        # - path[0, 0]: 경로의 맨 아래 x 좌표 (y=0일 때)
        base_diff = 320 - path[0, 0]
        # 경로 방향 계산: 1차 방정식 기울기를 구하고 1000배로 확대
        # - np.polyfit(y, x, 1): y에 대한 x의 기울기와 절편 반환
        # - [0]: 기울기만 추출, 1000배로 확대해 민감도 증가
        direction = np.polyfit(path[:, 1], path[:, 0], 1)[0] * 1000
        # 방향 보정: 음수/양수에 따라 반전 및 크기 조정
        # - 양수면 오른쪽 기울기 → 왼쪽 조향(-), 음수면 반대
        # - 0.4: 조향각 크기를 줄이는 스케일링 상수
        direction *= -0.4 if direction > 0 else -0.4
        # 방향 값이 클 경우 추가 보정: 과도한 조향 방지
        if abs(direction) > 5:
            # - 0.45 + (0.05 * (n)): 값이 클수록 점진적으로 감쇠
            # - (abs(direction) - 5) // 5: 5도 초과분을 5단위로 나눔
            direction *= 0.45 + (0.05 * ((abs(direction) - 5) // 5))
        # 조향각 제한: -20 ~ 20도 사이로 유지
        # - max/min: 범위 밖 값을 자름
        direction = max(min(direction, 20), -20)
        # 현재 조향각 저장: 다음 프레임에서 실패 시 사용
        self.steering_memory = direction
        # 결과 반환: 계산된 조향각
        return direction

    # 전체 처리 메서드: 이미지 입력부터 조향각 출력까지
    def process(self, origin_img):
        # 입력: 원본 카메라 이미지(origin_img)
        # 이미지 크기 조정: 입력 이미지를 640x480으로 변환
        # - cv2.INTER_LINEAR: 선형 보간으로 부드럽게 크기 조정
        origin_img = cv2.resize(origin_img, (640, 480), cv2.INTER_LINEAR)
        # 전처리: Camera 모듈로 이진화 이미지 생성
        # - Camera.pre_processing: 색상 필터링, 노이즈 제거 등으로 차선 강조
        img = self.camera.pre_processing(origin_img)
        # 슬라이딩 윈도우 실행: 차선 탐지 및 좌표 계산
        # - draw_windows=True: 디버깅용 윈도우 표시 활성화
        out_img, left_fitx, right_fitx, left_lane_detected, right_lane_detected = self.sliding_window(img, draw_windows=True)
        # 경로 생성: 좌우 차선 중간에 주행 경로 계산
        # - sliding_img에 경로 점 추가 (draw_windows=True)
        path_x, path_y = self.draw_path(out_img, left_fitx, right_fitx, draw_windows=True)
        # 조향각 계산: 경로 기반으로 방향 결정
        curvature_angle_new = self.get_angle(path_x, path_y, left_lane_detected, right_lane_detected)
        # 결과 반환: 계산된 조향각 (도 단위)
        return curvature_angle_new
