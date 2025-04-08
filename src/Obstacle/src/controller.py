import math

# Stanley 제어 클래스 (차량의 조향 제어용)
class Stanley:
    def __init__(self):
        # 조향 민감도를 조절하는 계수 (튜닝 대상)
        self.k = 0.9

    # 조향 각도를 계산하는 함수
    def control(self, middle_point, centerx, u, curvature_angle):
        psi = curvature_angle  # 차선 곡률로부터 추정한 조향각

        # 조향 각도 제한 (최대 ±20도)
        if psi >= 20.0:
            psi = 20.0
        elif psi <= -20.0:
            psi = -20.0

        return psi


# PID 제어 클래스 (속도 제어용 - 현재는 비례 제어만 사용)
class PID:
    def __init__(self):
        # 비례 제어 계수
        self.P_GAIN = 5.0

    # 비례 제어 계산
    def p_control_system(self, current_speed, target_speed):
        # 목표값과 현재값의 차이 (오차)
        error = target_speed - current_speed
        pControl = self.P_GAIN * error
        return pControl

    # 메인 PID 제어 함수 (현재는 P 제어만 적용)
    def pid_main(self, current_speed, target_speed):
        # P 제어 결과 계산
        pControl = self.p_control_system(current_speed, target_speed)

        # 제어 값을 모터에 맞는 범위로 변환 (비례 확장)
        controlled_speed = pControl * 12.5

        # 속도 상한 제한
        if controlled_speed >= 20.0:
            controlled_speed = 20.0

        return controlled_speed
