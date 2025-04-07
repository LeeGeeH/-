#!/usr/bin/env python3
# -*- coding:utf-8 -*-


import rospy
import cv2
from horse_power import HP

rospy.init_node('xytron_driving')
horse_power = HP()

if __name__ == '__main__':
	rospy.loginfo(rospy.get_name() + " started!")  # 노드 네임 출력
	while not rospy.is_shutdown():
		horse_power.control()
		if cv2.waitKey(1) & 0xff == ord('q'): # 주석처리
			break
	cv2.destroyAllWindows()
