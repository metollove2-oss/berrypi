"""2단계 모터 확인: 전진 · 좌회전 · 우회전 · 후진 · 자동 정지
반드시 바퀴를 띄운 상태(책 위)에서 실행: python3 test_motor.py
"""
import glob
import time

import serial

ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
if not ports:
    raise SystemExit("아두이노를 찾을 수 없음: USB 연결 확인 (ls /dev/ttyACM*)")
ser = serial.Serial(ports[0], 115200, timeout=1)
time.sleep(2)
ser.reset_input_buffer()
ser.write(b"PING\n")
print("연결:", ser.readline().decode(errors="ignore").strip(), "(PONG 이어야 함)")


def run(cmd, sec, name):
    """0.1초마다 같은 명령을 다시 보냄 (안 보내면 0.5초 뒤 아두이노가 멈춤)"""
    print(f"{name:8s} {cmd}")
    end = time.time() + sec
    while time.time() < end:
        ser.write((cmd + "\n").encode())
        time.sleep(0.1)


try:
    run("M,150,150", 1.5, "전진")
    run("M,75,150", 1.5, "좌회전")       # 오른쪽 바퀴가 더 빨라야 함
    run("M,150,75", 1.5, "우회전")       # 왼쪽 바퀴가 더 빨라야 함
    run("M,-150,-150", 1.5, "후진")
    run("M,0,0", 0.5, "정지")
    print("안전 장치: 전진 명령 한 번만 보냄 → 약 0.5초 뒤 저절로 서야 함")
    ser.write(b"M,150,150\n")
    time.sleep(1.5)
finally:
    ser.write(b"M,0,0\n")
    ser.close()
