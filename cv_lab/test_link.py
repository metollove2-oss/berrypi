"""1단계 연결 확인: 카메라 한 장 + 아두이노 PING/PONG (모터는 안 돎)
실행: python3 test_link.py
"""
import glob
import time

import serial
from picamera2 import Picamera2

# 1) 카메라
cam = Picamera2()
cam.configure(cam.create_preview_configuration(main={"size": (320, 240), "format": "RGB888"}))
cam.start()
time.sleep(1)
frame = cam.capture_array()
print("카메라 OK:", frame.shape, "평균 밝기", int(frame.mean()))
cam.stop()

# 2) 아두이노
ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
if not ports:
    raise SystemExit("아두이노를 찾을 수 없음: USB 연결 확인 (ls /dev/ttyACM*)")
ser = serial.Serial(ports[0], 115200, timeout=1)
time.sleep(2)                       # 포트를 열면 아두이노가 리셋되므로 기다림
ser.reset_input_buffer()
ser.write(b"PING\n")
print(f"아두이노({ports[0]}) 응답:", ser.readline().decode(errors="ignore").strip(), "(PONG 이어야 함)")
ser.write(b"D,94,78\n")             # 시험 명령: 모터는 안 돌고 계산한 PWM만 돌아옴
print("D 응답:", ser.readline().decode(errors="ignore").strip(), "(예: D F 169 F 160)")
ser.close()
