// 베리파이 모터 + 충돌 방지 펌웨어 (초안) — L9110S, HC-SR04, 피에조
//
// 파이 → 아두이노
//   M,<좌>,<우>        바퀴 세기 -255 ~ 255 (0.5초 안 오면 정지)
//   S                  즉시 정지
//   PING               → PONG
//   D,<좌>,<우>        (시험) 계산한 PWM만 회신: D F 169 F 160
//   ARM,<0|1>          경고음 켜기 조건 (파이가 주행 중 + 충돌 방지 켬 일 때 1)
//   MUTE,<0|1>         경고음 끄기
//   TH,<주의>,<위험>,<비상>  구간 거리 (mm), 예: TH,300,150,80
// 아두이노 → 파이
//   READY              부팅
//   R,<mm>             전방 거리, 100 ms마다 (-1 = 센서 응답 없음)
//   E,1 / E,0          비상 정지 시작 / 해제
//
// 역할: 측정 · 비상 정지 · 경고음은 아두이노, 감속 · 정지 판단 · 화면 경고는 파이.
// 비상 정지(기본 80 mm 미만)는 파이 명령과 상관없이 항상 동작한다.

// ---- 핀 ----
const int L_IA = 3,  L_IB = 11;   // 왼쪽 모터  (Timer2)
const int R_IA = 9,  R_IB = 10;   // 오른쪽 모터 (Timer1)
const int TRIG = 7,  ECHO = 8;    // HC-SR04
const int SPK = 5;                // 피에조 (D5 → 피에조 → GND)
const int LED = LED_BUILTIN;      // D13: M 명령 받는 중이면 켜짐

// ---- 모터 조정 값 ----
const int MIN_PWM = 120;
const int LEFT_BOOST = 0, RIGHT_BOOST = 0;
const int LEFT_DIR = 1, RIGHT_DIR = 1;
const unsigned long TIMEOUT_MS = 500;

// ---- 거리 설정 (TH 명령으로 바뀜) ----
int cautionMm = 300, dangerMm = 150, emergMm = 80;   // 파이가 TH로 다시 보냄
const int HYST_MM = 30;               // 구간에서 나올 때 여유 (깜빡임 방지)
const unsigned long MEASURE_MS = 60;  // 측정 주기
const unsigned long REPORT_MS = 100;  // R 보고 주기
const unsigned long ECHO_TIMEOUT_US = 12000;  // 약 2 m. 그 이상은 '장애물 없음'
const int FAIL_LIMIT = 10;            // 연속 무응답 → 센서 오류(-1)

// ---- 피에조 종류 ----
const bool ACTIVE_BUZZER = false;     // 5V만 대도 '삐-' 울리는 능동형이면 true

// ===================== 모터 =====================
unsigned long lastCmd = 0;
bool stopped = true;
int cmdL = 0, cmdR = 0;               // 파이가 마지막으로 보낸 값

int toPwm(int mag, int boost) {
  mag = constrain(mag, 0, 255);
  if (mag == 0) return 0;
  return constrain((int)map(mag, 1, 255, MIN_PWM, 255) + boost, 0, 255);
}

void setMotor(int ia, int ib, int speed, int boost) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(ia, toPwm(speed, boost)); digitalWrite(ib, LOW); }
  else if (speed < 0) { digitalWrite(ia, LOW); analogWrite(ib, toPwm(-speed, 0)); }
  else                { digitalWrite(ia, LOW); digitalWrite(ib, LOW); }
}

bool emergency = false;

void drive(int left, int right) {
  if (emergency) { left = 0; right = 0; }      // 비상 정지 중에는 항상 0
  setMotor(L_IA, L_IB, left * LEFT_DIR, LEFT_BOOST);
  setMotor(R_IA, R_IB, right * RIGHT_DIR, RIGHT_BOOST);
  stopped = (left == 0 && right == 0);
  digitalWrite(LED, stopped ? LOW : HIGH);
}

// ===================== 경고음 (논블로킹) =====================
struct Note { int freq; int ms; };              // freq 0 = 쉼
const Note RIFF_CAUTION[] = {{1800, 80}, {2400, 80}, {0, 640}};   // 띠-리 ... (0.8초 반복)
const Note RIFF_DANGER[]  = {{2600, 70}, {2000, 70}};             // 사이렌 (계속 반복)
const Note RIFF_EMERG[]   = {{3000, 200}};                        // 높은 음 계속
const Note RIFF_CLEAR[]   = {{2400, 60}, {1800, 60}, {1200, 100}, {0, 1}};  // 해제 (한 번)

const Note *riff = 0;
int riffLen = 0, noteIdx = 0;
bool riffLoop = false;
unsigned long noteStart = 0, lastToggle = 0;
bool spkOn = false;

void playRiff(const Note *r, int len, bool loop) {
  if (riff == r) return;                       // 이미 재생 중이면 그대로
  riff = r; riffLen = len; riffLoop = loop; noteIdx = 0;
  noteStart = millis(); lastToggle = micros();
}

void stopRiff() {
  riff = 0;
  digitalWrite(SPK, LOW);                      // 쉴 때는 꼭 LOW
  spkOn = false;
}

void updateSound() {
  if (!riff) return;
  unsigned long now = millis();
  if (now - noteStart >= (unsigned long)riff[noteIdx].ms) {   // 다음 음
    noteIdx++;
    noteStart = now;
    if (noteIdx >= riffLen) {
      if (riffLoop) noteIdx = 0;
      else { stopRiff(); return; }
    }
  }
  int f = riff[noteIdx].freq;
  if (f == 0) { digitalWrite(SPK, LOW); spkOn = false; return; }
  if (ACTIVE_BUZZER) { digitalWrite(SPK, HIGH); return; }       // 능동형: 켜기만
  unsigned long half = 500000UL / f;                            // 반 주기 (µs)
  unsigned long t = micros();
  if (t - lastToggle >= half) {
    spkOn = !spkOn;
    digitalWrite(SPK, spkOn ? HIGH : LOW);
    lastToggle = t;
  }
}

// ===================== 초음파 =====================
int buf3[3] = {-1, -1, -1};
int bufIdx = 0;
int distMm = -1;            // 중간값 (mm), -1 = 무응답
int failCount = 0;
int emergCount = 0, clearCount = 0;
bool armed = false, muted = false;
enum Zone { SAFE, CAUTION, DANGER, EMERG };
Zone zone = SAFE;
void updateRiffForZone(Zone before);   // Arduino 자동 선언이 enum 앞에 생기는 문제 방지

const int FAR = 9999;       // 2 m 안에 장애물 없음
const int NORESP = -1;      // 센서 응답 없음 (선 빠짐 등)

int readOnceMm() {
  if (digitalRead(ECHO) == HIGH) return FAR;   // 지난 측정의 '먼 거리' 펄스가 아직 진행 중
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) {
    // ECHO가 HIGH인 채로 시간 초과 = 반사파가 늦음 = 멀다. 한 번도 HIGH가 안 됨 = 센서 응답 없음
    return digitalRead(ECHO) == HIGH ? FAR : NORESP;
  }
  return (int)(us * 343UL / 2000UL);           // µs → mm (음속 343 m/s)
}

int median3(int a, int b, int c) {
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}

void measure() {
  int d = readOnceMm();
  if (d == NORESP) failCount++; else failCount = 0;
  buf3[bufIdx] = (d == NORESP) ? FAR : d;      // 한두 번 무응답은 '멀다'로 취급
  bufIdx = (bufIdx + 1) % 3;
  int m = median3(buf3[0], buf3[1], buf3[2]);  // 3회 중간값 (튀는 값 제거)
  distMm = (failCount >= FAIL_LIMIT) ? -1 : (m >= FAR ? 2000 : m);   // 2000 = 2 m 이상

  // 비상 정지: 3회 연속 emergMm 미만 → 정지, 3회 연속 emergMm + HYST 초과 → 해제
  bool valid = distMm > 0;
  if (valid && distMm < emergMm) { emergCount++; clearCount = 0; }
  else if (!valid || distMm > emergMm + HYST_MM) { clearCount++; emergCount = 0; }
  if (!emergency && emergCount >= 3) {
    emergency = true;
    drive(0, 0);
    Serial.println("E,1");
  } else if (emergency && clearCount >= 3) {
    emergency = false;
    Serial.println("E,0");
  }

  // 경고음 구간 (들어갈 때는 기준 거리, 나올 때는 기준 + HYST_MM)
  Zone z;
  if (!valid)                                           z = SAFE;
  else if (distMm < emergMm)                            z = EMERG;
  else if (distMm < dangerMm)                           z = DANGER;
  else if (distMm < dangerMm + HYST_MM && zone >= DANGER)   z = DANGER;
  else if (distMm < cautionMm)                          z = CAUTION;
  else if (distMm < cautionMm + HYST_MM && zone >= CAUTION) z = CAUTION;
  else                                                  z = SAFE;
  if (emergency) z = EMERG;
  Zone before = zone;
  zone = z;
  updateRiffForZone(before);
}

void updateRiffForZone(Zone before) {
  if (muted || !armed) {
    if (riff && riff != RIFF_CLEAR) stopRiff();
    if (!emergency || muted) return;           // 비상은 armed와 상관없이 울림 (mute 제외)
  }
  switch (zone) {
    case EMERG:   playRiff(RIFF_EMERG, 1, true); break;
    case DANGER:  playRiff(RIFF_DANGER, 2, true); break;
    case CAUTION: playRiff(RIFF_CAUTION, 3, true); break;
    case SAFE:
      if (before >= DANGER) playRiff(RIFF_CLEAR, 4, false);   // 위험에서 벗어나면 해제음
      else if (riff && riff != RIFF_CLEAR) stopRiff();
      break;
  }
}

// ===================== 통신 =====================
void reportSide(int speed, int boost) {
  speed = constrain(speed, -255, 255);
  Serial.print(speed > 0 ? " F " : (speed < 0 ? " B " : " S "));
  Serial.print(speed > 0 ? toPwm(speed, boost) : toPwm(-speed, 0));
}

void handleLine(char *line) {
  int a, b, c;
  if (line[0] == 'M' && sscanf(line, "M,%d,%d", &a, &b) == 2) {
    cmdL = a; cmdR = b;
    drive(a, b);
    lastCmd = millis();
  } else if (strcmp(line, "PING") == 0) {
    Serial.println("PONG");
  } else if (strcmp(line, "S") == 0) {
    drive(0, 0);
  } else if (line[0] == 'D' && sscanf(line, "D,%d,%d", &a, &b) == 2) {
    Serial.print("D"); reportSide(a, LEFT_BOOST); reportSide(b, RIGHT_BOOST); Serial.println();
  } else if (sscanf(line, "ARM,%d", &a) == 1) {
    armed = (a != 0);
    updateRiffForZone(zone);
  } else if (sscanf(line, "MUTE,%d", &a) == 1) {
    muted = (a != 0);
    updateRiffForZone(zone);
  } else if (sscanf(line, "TH,%d,%d,%d", &a, &b, &c) == 3) {
    if (c > 0 && b > c && a > b) { cautionMm = a; dangerMm = b; emergMm = c; }
  }
}

char line[32];
int len = 0;

void readLine() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { line[len] = '\0'; handleLine(line); len = 0; }
    else if (len < 31) line[len++] = ch;
  }
}

// ===================== setup / loop =====================
unsigned long lastMeasure = 0, lastReport = 0;

void setup() {
  Serial.begin(115200);
  int outs[] = {L_IA, L_IB, R_IA, R_IB, TRIG, SPK, LED};
  for (int p : outs) pinMode(p, OUTPUT);
  pinMode(ECHO, INPUT);
  drive(0, 0);
  digitalWrite(SPK, LOW);
  Serial.println("READY");
}

void loop() {
  readLine();
  unsigned long now = millis();
  if (now - lastMeasure >= MEASURE_MS) { measure(); lastMeasure = now; }
  if (now - lastReport >= REPORT_MS) {
    Serial.print("R,"); Serial.println(distMm);
    lastReport = now;
  }
  if (!stopped && now - lastCmd > TIMEOUT_MS) drive(0, 0);   // 명령 끊김 → 정지
  updateSound();
}
