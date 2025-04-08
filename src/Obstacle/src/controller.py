import math
import numpy as np

class Stanley:
    # Stanley 제어기: 차선 중심선과 차량 중심 간의 횡방향 오차와 곡률 정보를 사용해 조향각 계산
    # x: 횡방향 오차 (meter)
    # u: 종방향 속도 (m/s)
    # curvature_angle: 차선 인식으로 측정된 곡률 각도 값 (degree)
    
    def __init__(self):
        # self.k: Stanley 제어기의 게인 값 (크면 반응이 민감해짐)
        self.k = 0.9
        
    def calc_x(self, middle_point, centerx):
        # 이미지 상의 픽셀 좌표를 실제 차량 기준의 횡방향 거리로 변환
        # middle_point: 차선 중앙 좌표 (픽셀)
        # centerx: 차량 이미지 기준 중심 (픽셀)
        x = (centerx - middle_point) * 0.0001  # 픽셀 -> meter로 단순 변환 (보정 필요 가능)
        return x

    def control(self, middle_point, centerx, u, curvature_angle):
        # Stanley 제어기 메인 함수
        # middle_point: 인식된 차선 중심점
        # centerx: 차량 중심점
        # u: 현재 속도
        # curvature_angle: 곡률 기반의 방향 각도

        psi = curvature_angle  # 곡률에서 계산된 조향각도
        x = self.calc_x(middle_point, 340)  # 횡방향 오차 계산 (차량 중심은 340픽셀로 가정)

        try:
            cte = 0  # 횡방향 오차 (Cross Track Error), 현재는 비활성화
        except:
            cte = 0

        stanley_angle = psi + cte  # 조향각 = 진행 방향 오차 + 횡방향 오차 보정

        # 조향 각도 제한 (안전/제어 안정성 확보용)
        if stanley_angle >= 20.0:
            stanley_angle = 20.0
        elif stanley_angle <= -20.0:
            stanley_angle = -20.0

        return stanley_angle


class PID:
    # PID 제어기: 목표 속도와 현재 속도 간 오차를 기반으로 속도를 보정
    def __init__(self):
        # PID 제어 이득 값 (필요시 튜닝)
        self.P_GAIN = 5.0
        self.I_GAIN = 0.0
        self.D_GAIN = 0.0

        # 제어용 변수 초기화
        self.error, self.acc_error = 0.0, 0.0           # 현재 오차, 누적 오차
        self.prev_error, self.error_gap = 0.0, 0.0      # 이전 오차, 오차 변화량

        # 각 제어 항목 결과값
        self.pControl, self.iControl = 0.0, 0.0
        self.dControl, self.pidControl = 0.0, 0.0

        self.time = 0.033  # 제어 주기 (30Hz 기준)

    def conv_to_real_speed(self, motor_val):
        # 모터 속도 값을 실제 m/s 단위로 변환
        if motor_val >= 20.0:
            motor_val = 20.0  # 최대 속도 제한
        real_speed = motor_val * 0.08  # 비례 상수 (속도 환산)
        return real_speed

    def p_control_system(self, current_speed, target_speed):
        # 비례 제어 (P 제어)
        self.error = self.conv_to_real_speed(target_speed) - current_speed  # 목표 - 현재
        self.pControl = self.P_GAIN * self.error

    def i_control_system(self):
        # 적분 제어 (I 제어)
        self.acc_error += self.error  # 누적 오차 갱신
        self.iControl = self.I_GAIN * self.acc_error * self.time

    def d_control_system(self):
        # 미분 제어 (D 제어)
        self.error_gap = self.error - self.prev_error  # 오차의 변화율
        self.dControl = self.D_GAIN * (self.error_gap / self.time)
        self.prev_error = self.error  # 이전 오차 업데이트

    def pid_control_system(self):
        # 전체 PID 제어 출력 계산
        self.pidControl = self.pControl + self.iControl + self.dControl

    def pid_main(self, current_speed, target_speed):
        # PID 제어 실행 함수
        self.p_control_system(current_speed, target_speed)
        self.i_control_system()
        self.d_control_system()
        self.pid_control_system()

        controlled_speed = self.pidControl * 12.5  # 다시 모터 속도 값으로 환산 (real_speed → motor_val)

        # 최대 속도 제한
        if controlled_speed >= 20.0:
            controlled_speed = 20.0

        return controlled_speed
