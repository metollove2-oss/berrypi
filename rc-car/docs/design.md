# 설계 요약

설계서 원본: (설계서 링크)

- 상위 제어부: Raspberry Pi 5 — 카메라, 전처리, CNN(TFLite) 추론, 속도 명령 송신
- 하위 제어부: Arduino Uno + L293D — 모터 차동 조향, 초음파 정지, LCD, 가변저항
- 전원: 보조배터리 → 파이 (아두이노는 파이 USB로 급전), 배터리팩 → L293D VCC2, GND 공통

## 아두이노 핀 배치

| 핀 | 연결 |
| --- | --- |
| D0, D1 | 파이 USB 시리얼 (다른 용도 금지) |
| D5 (PWM) / D2, D4 | L293D EN1 / IN1, IN2 — 좌 모터 |
| D6 (PWM) / D7, D8 | L293D EN2 / IN3, IN4 — 우 모터 |
| D12, D13 | HC-SR04 Trig, Echo |
| D3 | Serial LCD Rx (SoftwareSerial) |
| A2 | 가변저항 |
