#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy  # ROS 파이썬 라이브러리
import cv2  # OpenCV 라이브러리 (이미지 처리)
from stopline_detector import StoplineDetector  # 정지선 감지 모듈
from camera import Camera  # 카메라 데이터 처리 모듈
from horse_power import HP  # 차량 제어 모듈
import numpy as np  # 수치 연산 라이브러리
import subprocess  # 외부 명령 실행 라이브러리

# ========================================
# 아두이노 코드 업로드 함수
# 입력: 아두이노 .ino 파일 경로
# 기능: 아두이노 CLI를 사용해 코드 업로드
# ========================================
def upload_arduino_code(ino_file):
    # 아두이노 CLI 명령어 설정: UNO 보드에 업로드
    command2 = [
        "arduino-cli", "upload", "-p", "/dev/ttyACM0", "--fqbn", "arduino:avr:uno", ino_file
    ]
    subprocess.run(command2)  # 명령어 실행

# ========================================
# 아두이노 코드 컴파일 및 업로드 함수
# 입력: 아두이노 .ino 파일 경로
# 기능: 아두이노 CLI를 사용해 컴파일 후 업로드
# ========================================
def compile_arduino_code(ino_file):
    # 아두이노 CLI 명령어 설정: Mega 보드에 컴파일 및 업로드
    command3 = [
        "arduino-cli", "compile", "--fqbn", "arduino:avr:mega:cpu=atmega2560", "-u", "-p", "/dev/ttyACM0", ino_file
    ]
    subprocess.run(command3)  # 명령어 실행

    # 주석 처리된 대체 명령어 (컴파일과 업로드 분리)
    '''
    command1 = [  # 컴파일만
        "arduino-cli", "compile", "--fqbn", "arduino:avr:uno", ino_file
    ]
    subprocess.run(command1)

    command2 = [  # 업로드만
        "arduino-cli", "upload", "-p", "/dev/ttyACM0", "--fqbn", "arduino:avr:uno", ino_file
    ]
    subprocess.run(command2)
    '''

# ROS 노드 초기화: 'obstacle_node'라는 이름으로 노드 생성
rospy.init_node('obstacle_node')

# 차량 제어 객체 초기화
horse_power = HP()

# ========================================
# 메인 실행 블록
# 기능: ROS 노드 실행 및 제어 루프
# ========================================
if __name__ == '__main__':
    rospy.loginfo(rospy.get_name() + " started!")
    while not rospy.is_shutdown():
        horse_power.control()
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cv2.destroyAllWindows()