"""OpenCV 차선 검출 + PD 제어 (기준 방식이자 데이터 수집용 자동 라벨러)

TODO (팀원 1)
- detect_center(bgr): 그레이스케일 → 이진화 → ROI → 차선 중심 x 좌표 반환 (못 찾으면 None)
- PDController.update(error): 오차(-1~+1) → 조향값(-1~+1)
"""
from pi import config


def detect_center(bgr):
    raise NotImplementedError


class PDController:
    def __init__(self, kp=config.KP, kd=config.KD):
        self.kp, self.kd, self.prev = kp, kd, 0.0

    def update(self, error):
        d = error - self.prev
        self.prev = error
        return max(-1.0, min(1.0, self.kp * error + self.kd * d))
