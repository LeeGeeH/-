import math

# Stanley 제어 클래스 (차량의 조향 제어용)
class Stanley:
    
    # 조향 각도를 계산하는 함수
    def control(self, middle_point, centerx, u, curvature_angle):
        psi = curvature_angle  # 차선 곡률로부터 추정한 조향각

        # 조향 각도 제한 (최대 ±20도)
        if psi >= 20.0:
            psi = 20.0
        elif psi <= -20.0:
            psi = -20.0

        return psi
