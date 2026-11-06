// 하위 제어부 펌웨어 — 통신 규칙은 docs/protocol.md, 핀 배치는 docs/design.md
// TODO (팀원 2): parseCommand, setMotors, readUltrasonic, updateLCD 구현
#include <SoftwareSerial.h>

// L293D
const int EN_L = 5, IN1 = 2, IN2 = 4;   // 좌 모터
const int EN_R = 6, IN3 = 7, IN4 = 8;   // 우 모터
// HC-SR04
const int TRIG = 12, ECHO = 13;
// Serial LCD (Rx 연결), 가변저항
SoftwareSerial lcd(255, 3);             // RX 미사용, TX = D3
const int POT = A2;

const long STOP_MM = 200;               // 장애물 정지 거리
const unsigned long CMD_TIMEOUT = 500;  // 명령 끊김 정지 (ms)

int cmdL = 0, cmdR = 0;
unsigned long lastCmd = 0;

void setup() {
  Serial.begin(115200);
  lcd.begin(9600);
  int pins[] = {EN_L, IN1, IN2, EN_R, IN3, IN4, TRIG};
  for (int p : pins) pinMode(p, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  // 1) 명령 수신: "L,180,R,95" 또는 "S"
  // 2) 거리 측정 (pulseIn에 timeout 지정)
  // 3) 가변저항 → 최대 PWM
  // 4) 정지 조건 확인 후 setMotors
  // 5) 200ms마다 LCD 갱신
  // delay() 사용 금지 — millis()로 시간 관리
}
