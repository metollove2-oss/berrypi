"""아두이노 시리얼 통신 — 규칙은 docs/protocol.md"""
import time

import serial

from pi import config


class ArduinoLink:
    def __init__(self, port=config.SERIAL_PORT, baud=config.BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(2)  # 포트를 열면 우노가 리셋되므로 부팅 대기

    def send_speed(self, left, right):
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        self.ser.write(f"L,{left},R,{right}\n".encode())

    def stop(self):
        self.ser.write(b"S\n")

    def read_line(self):
        """아두이노가 보낸 상태 메시지 한 줄 (없으면 None)"""
        line = self.ser.readline().decode(errors="ignore").strip()
        return line or None

    def close(self):
        self.stop()
        self.ser.close()


def steer_to_speed(s, base_pwm=config.BASE_PWM):
    """조향값 s(-1~+1) → 좌/우 PWM"""
    s = max(-1.0, min(1.0, s))
    base = base_pwm * (1 - config.TURN_SLOWDOWN * abs(s))
    k = config.STEER_GAIN
    return base * (1 + k * s), base * (1 - k * s)
