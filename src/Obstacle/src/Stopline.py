#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy          # ROS 파이썬 라이브러리
import cv2           # OpenCV 라이브러리 (이미지 처리)
from horse_power import HP  # 차량 제어 모듈

# ROS 노드 및 차량 제어 객체 초기화
rospy.init_node('obstacle_node')
horse_power = HP()

# 메인 실행 블록
if __name__ == '__main__':
    rospy.loginfo(f"{rospy.get_name()} started!")
    while not rospy.is_shutdown():
        horse_power.control()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
