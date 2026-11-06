"""주행 — 카메라 → (OpenCV | CNN) → 조향값 → 좌/우 PWM → 아두이노

TODO (팀원 1)
- --mode opencv : lane_opencv.detect_center + PDController
- --mode cnn    : common.preprocess.preprocess → TFLite(config.MODEL_PATH) 추론
- 카메라 캡처는 별도 스레드, 종료 시 link.close()로 정지
실행: python -m pi.drive --mode cnn
"""
