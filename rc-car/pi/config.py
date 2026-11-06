"""주행 튜닝 값 — 트랙에서 바꾸면 그 자리에서 커밋할 것"""

# 시리얼
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 115200
SEND_HZ = 20

# 속도 (PWM 0~255, 실제 최대치는 아두이노 가변저항으로 한 번 더 제한)
BASE_PWM = 160
TURN_SLOWDOWN = 0.5   # base = BASE_PWM * (1 - TURN_SLOWDOWN * |s|)
STEER_GAIN = 1.0      # 좌 = base*(1 + K*s), 우 = base*(1 - K*s)

# OpenCV 차선 검출 + PD 제어
KP = 0.8
KD = 0.2

# CNN
MODEL_PATH = "models/pilotnet.tflite"

# 데이터 수집
DATA_DIR = "data"
SAVE_HZ = 10
