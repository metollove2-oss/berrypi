// 베리파이 모터 펌웨어 (강의 10. motor_lab 인터페이스와 같음) — L9110S 기준
// 파이 → 아두이노:  M,<좌>,<우>  (-255 ~ 255)   PING   S   D,<좌>,<우>(시험)
// 아두이노 → 파이:  READY  PONG  D F 169 F 160
// 0.5초 동안 M 명령이 없으면 정지. 명령을 받을 때마다 보드 LED(D13)가 켜짐.

// ---- 핀 (L9110S) ----
const int L_IA = 3,  L_IB = 11;   // 왼쪽 모터  (Timer2, 490Hz)
const int R_IA = 9,  R_IB = 10;   // 오른쪽 모터 (Timer1, 490Hz)
const int LED = LED_BUILTIN;      // D13: M 명령을 받으면 켜짐 (통신 확인용)

// ---- 조정 값 ----
const int MIN_PWM = 120;          // 바퀴가 출발하는 최소 PWM (안 돌면 올리기)
const int LEFT_BOOST = 0;         // 직진 시 왼쪽이 느리면 올리기
const int RIGHT_BOOST = 0;        // 직진 시 오른쪽이 느리면 올리기
const int LEFT_DIR = 1;           // 왼쪽 바퀴가 반대로 돌면 -1
const int RIGHT_DIR = 1;          // 오른쪽 바퀴가 반대로 돌면 -1
const unsigned long TIMEOUT_MS = 500;

unsigned long lastCmd = 0;
bool stopped = true;
char buf[32];
int len = 0;

// 명령 크기 0~255 → PWM (0은 정지, 1~255 → MIN_PWM~255)
int toPwm(int mag, int boost) {
  mag = constrain(mag, 0, 255);
  if (mag == 0) return 0;                       // 0은 map에 넣지 않음
  int pwm = map(mag, 1, 255, MIN_PWM, 255) + boost;
  return constrain(pwm, 0, 255);
}

// speed 부호 = 방향, 크기 = 세기
void setMotor(int ia, int ib, int speed, int boost) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(ia, toPwm(speed, boost));
    digitalWrite(ib, LOW);
  } else if (speed < 0) {
    digitalWrite(ia, LOW);
    analogWrite(ib, toPwm(-speed, 0));
  } else {
    digitalWrite(ia, LOW);
    digitalWrite(ib, LOW);
  }
}

void drive(int left, int right) {
  setMotor(L_IA, L_IB, left * LEFT_DIR, LEFT_BOOST);
  setMotor(R_IA, R_IB, right * RIGHT_DIR, RIGHT_BOOST);
  stopped = (left == 0 && right == 0);
  digitalWrite(LED, stopped ? LOW : HIGH);
}

// D 시험 명령 응답: "D F 169 F 160"
void reportSide(int speed, int boost) {
  speed = constrain(speed, -255, 255);
  Serial.print(speed > 0 ? " F " : (speed < 0 ? " B " : " S "));
  Serial.print(speed > 0 ? toPwm(speed, boost) : toPwm(-speed, 0));
}

void handleLine(char *line) {
  int l, r;
  if (line[0] == 'M' && sscanf(line, "M,%d,%d", &l, &r) == 2) {
    drive(l, r);
    lastCmd = millis();
  } else if (strcmp(line, "PING") == 0) {
    Serial.println("PONG");
  } else if (strcmp(line, "S") == 0) {
    drive(0, 0);
  } else if (line[0] == 'D' && sscanf(line, "D,%d,%d", &l, &r) == 2) {
    Serial.print("D");
    reportSide(l, LEFT_BOOST);
    reportSide(r, RIGHT_BOOST);
    Serial.println();
  }
  // 그 밖의 줄은 무시
}

// 글자를 모아 \n 이 오면 한 줄 처리 (\r 무시, 최대 31글자)
void readLine() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      handleLine(buf);
      len = 0;
    } else if (len < 31) {
      buf[len++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  int pins[] = {L_IA, L_IB, R_IA, R_IB, LED};
  for (int p : pins) pinMode(p, OUTPUT);
  drive(0, 0);
  Serial.println("READY");
}

void loop() {
  readLine();
  if (!stopped && millis() - lastCmd > TIMEOUT_MS) {
    drive(0, 0);                                // 명령 끊김 → 정지
  }
}
