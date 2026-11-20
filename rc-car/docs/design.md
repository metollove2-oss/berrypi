# 설계 요약

설계서 원본: (설계서 링크)

- 상위 제어부: Raspberry Pi 5 — 카메라, 전처리, CNN(TFLite) 추론, 속도 명령 송신
- 하위 제어부: Arduino Uno + L293D — 모터 차동 조향, 초음파 정지, LCD, 가변저항
- 전원: 보조배터리 → 파이 (아두이노는 파이 USB로 급전), 배터리팩 → L293D VCC2, GND 공통
- 통신 규칙: `docs/protocol.md` · 데이터 형식: `docs/data_format.md`

## 아두이노 핀 배치 (v1, 2026-11-20)

| 핀 | 연결 | 비고 |
| --- | --- | --- |
| D0, D1 | 파이 USB 시리얼 | 다른 용도 금지 |
| **D9** (PWM) | L293D EN1 (1번) — 좌 모터 속도 | 설계서 D5에서 이동 |
| D2, D4 | L293D IN1, IN2 (2, 7번) — 좌 모터 방향 | |
| **D10** (PWM) | L293D EN2 (9번) — 우 모터 속도 | 설계서 D6에서 이동 |
| D7, D8 | L293D IN3, IN4 (10, 15번) — 우 모터 방향 | |
| D12, D13 | HC-SR04 Trig, Echo | |
| D3 | Serial LCD Rx (SoftwareSerial TX) | |
| A2 | 가변저항 Wiper | |

**EN 핀을 D5/D6 → D9/D10으로 옮긴 이유** (실습 10 참고)
- D5/D6는 Timer0(약 980 Hz)으로 `millis()`와 같은 타이머를 쓴다.
- D9/D10은 둘 다 Timer1(약 490 Hz)이라 좌우 PWM 주파수가 같고 시스템 시간과 분리된다.
- L293D는 EN에만 PWM이 필요하므로 IN 핀(D2, D4, D7, D8)과 나머지 배선은 그대로 둔다.
