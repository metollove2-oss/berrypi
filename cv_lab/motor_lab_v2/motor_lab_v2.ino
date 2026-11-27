/*
 * motor_lab_v2.ino - 모터 인터페이스 짝 코딩 실습 [아두이노 담당]
 *   v2 변경: MAX_PWM 추가 (웹 speed로 속도가 잘 줄도록 PWM 범위 조정),
 *            M 명령마다 출력하던 MOTOR 줄을 VERBOSE로 끄고 켜기
 *
 * 할 일: [A1] toPwm, setMotor   [A3] handleLine   [A4] loop의 안전 장치
 * 나머지 [제공] 부분은 고치지 않음. 함수 하나를 끝낼 때마다 업로드해서 시리얼 모니터로 시험.
 *
 * 약속 (Pi ↔ Arduino)
 *   115200bps, 한 명령 = 한 줄, 끝은 '\n'
 *   M,<왼쪽>,<오른쪽>   두 바퀴 세기 -255 ~ 255 (음수 = 후진, 0 = 정지)
 *   PING                → PONG
 *   D,<왼쪽>,<오른쪽>   (시험용) 모터는 돌리지 않고 계산한 방향·PWM만 출력  예: D F 169 F 160
 *   0.5초 동안 M 명령이 없으면 자동 정지
 *
 * 매개변수 (M, D 공통)
 *   <왼쪽>, <오른쪽> : 정수(int, 10진수, +/- 부호 가능), 유효 범위 -255 ~ 255
 *                      범위 밖은 ±255로 잘라서 처리, 소수 · 16진수 · 숫자 1개뿐인 줄은 무시
 *   명령 글자는 대문자 M / D / PING, 줄 맨 앞에 공백 없이, 한 줄 최대 31글자
 *
 * 배선 (L9110S) : 주파수가 같은 핀 사용 (490Hz)
 *   D3 = A-IA, D11 = A-IB (왼쪽)    D9 = B-IA, D10 = B-IB (오른쪽)    GND 공통
 */

const int L_IA = 3,  L_IB = 11;    // 왼쪽 바퀴
const int R_IA = 9,  R_IB = 10;    // 오른쪽 바퀴
const int MIN_PWM = 120;           // 바퀴가 출발하는 PWM. 두 바퀴가 같이 겨우 도는 값으로 낮출수록 느린 속도가 잘 됨 (예: 95~110)
const int MAX_PWM = 190;           // 명령 255일 때의 PWM (최고 속도). 낮출수록 전체가 느려짐. 원래 동작 = 255
const bool VERBOSE = false;        // true면 M 명령마다 "MOTOR LEFT: … RIGHT: …" 출력 (디버깅용)
const unsigned long TIMEOUT_MS = 500;
const int LEFT_DIR = 1, RIGHT_DIR = 1;     // 바퀴가 반대로 돌면 -1
const int LEFT_BOOST = 0, RIGHT_BOOST = 0; // 전진 시 한쪽 모터가 약하면 5~15로 보정

char buf[32];
int len = 0;
unsigned long lastCmd = 0;   // 마지막 M 명령을 받은 시각 (ms)
bool stopped = true;         // 지금 두 바퀴가 멈춰 있는지


int toPwm(int mag, int boost = 0) {
  if (mag <= 0) return 0;                                   // 0은 정지 (map에 넣지 않음)
  mag = constrain(mag, 1, 255);

  int startPwm = constrain(MIN_PWM + boost, 0, 255);        // 명령 1일 때
  int topPwm   = constrain(MAX_PWM + boost, startPwm, 255); // 명령 255일 때

  return startPwm + (long)(mag - 1) * (topPwm - startPwm) / 254;
}// mag(1~255)를 실제 PWM(startPwm~topPwm)으로 선형 변환

void setMotor(int ia, int ib, int speed, int boost = 0) {
  speed = constrain(speed, -255, 255);

  // 전진
  if (speed > 0) {
    int pwm = toPwm(speed, boost);
    analogWrite(ia, pwm);
    digitalWrite(ib, LOW);
  }

  // 후진
  else if (speed < 0) {
    int pwm = toPwm(abs(speed), 0);
    digitalWrite(ia, LOW);
    analogWrite(ib, pwm);
  }

  // 정지
  else {
    digitalWrite(ia, LOW);
    digitalWrite(ib, LOW);
  }
}

void drive(int left, int right) {
  setMotor(L_IA, L_IB, left * LEFT_DIR, LEFT_BOOST);
  setMotor(R_IA, R_IB, right * RIGHT_DIR, RIGHT_BOOST);
  stopped = (left == 0 && right == 0);
}

void handleLine(char *line) {
  int left, right;

  //모터 구동
  if (sscanf(line, "M,%d,%d", &left, &right) == 2) {
    drive(left, right);
    //시간 기록
    lastCmd = millis();

    // 모터 명령 수신 결과 출력 (VERBOSE일 때만: 초당 20번씩 나와 다른 메시지를 가리므로)
    if (VERBOSE) {
      Serial.print("MOTOR LEFT: ");
      Serial.print(left);
      Serial.print(" RIGHT: ");
      Serial.println(right);
    }
  } 

  //통신 확인
  else if (strcmp(line, "PING") == 0) {
    Serial.println("PONG");
  }
}

//시험용 : 모터 구동 없이 결과만 출력
void printWheel(int s, int boost = 0) {
  s = constrain(s, -255, 255);

  if (s > 0) {
    Serial.print("F ");
    Serial.print(toPwm(s, boost));
  }
  else if (s < 0) {
    Serial.print("B ");
    Serial.print(toPwm(-s, 0));
  }
  else {
    Serial.print("S ");
    Serial.print(toPwm(0));
  } 
   // 0이면 toPwm(0)도 0이어야 함
}

// 디버그 명령 처리
bool debugLine(char *line) {
  int l, r;

  if (sscanf(line, "D,%d,%d", &l, &r) != 2)
    return false;

  Serial.print("D ");

  printWheel(l * LEFT_DIR, LEFT_BOOST);
  Serial.print(" ");
  printWheel(r * RIGHT_DIR, RIGHT_BOOST);

  Serial.println();

  return true;
}

// [제공] 글자를 모아 한 줄이 되면 true (A2)
bool readLine() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      buf[len] = '\0';
      len = 0;
      return true;
    }
    else if (c != '\r' && len < (int)sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  return false;
}

void setup() {
  int pins[] = {L_IA, L_IB, R_IA, R_IB};
  for (int i = 0; i < 4; i++) pinMode(pins[i], OUTPUT);

  drive(0, 0);
  lastCmd = millis();

  Serial.begin(115200);
  Serial.println("READY");
}

void loop() {
  if (readLine()) {
    if (!debugLine(buf)) handleLine(buf);
  }

  if(!stopped && (millis() - lastCmd >= TIMEOUT_MS)){
    drive(0, 0);
  }
}