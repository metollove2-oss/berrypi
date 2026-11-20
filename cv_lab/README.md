# cv_lab — OpenCV 선 따라가기 (베리파이)

강의 11. cv_lab 기준 완성본. 카메라 · 웹 화면 · 모터 · 주행 반복 · C1~C5가 `cv_lab.py` 하나에 모두 들어 있다.
아두이노에는 강의의 `motor_serial.ino` (또는 `motor_lab.ino`)가 올라가 있어야 한다.

| 파일 | 내용 |
| --- | --- |
| `cv_lab.py` | 주행 프로그램 (이것만 실행하면 됨) |
| `check_cv.py` | C1~C5 자동 확인 21개 (카메라 · 모터 없이) |
| `test_link.py` | 카메라 + 아두이노 PING/PONG 확인 (모터 안 돎) |
| `test_motor.py` | 바퀴 전진 · 좌회전 · 우회전 · 후진 · 자동 정지 확인 |

## 1. 파이에 받기 (처음 한 번)

```bash
sudo apt install -y python3-opencv python3-numpy python3-flask python3-picamera2 python3-serial python3-venv git
cd ~
git clone https://github.com/metollove2-oss/berrypi.git
cd ~/berrypi/cv_lab
python3 -m venv --system-site-packages venv
```

이미 받았으면 `cd ~/berrypi && git pull` 로 최신 코드만 받는다.

## 2. 매번 실행

```bash
cd ~/berrypi/cv_lab
source venv/bin/activate        # 줄 앞에 (venv) 가 붙으면 OK

python3 check_cv.py             # 통과 21 / 21
python3 test_link.py            # 카메라 OK + PONG
python3 test_motor.py           # 바퀴 띄우고! 5가지 동작 확인
python3 cv_lab.py               # 주행
```

`cv_lab.py`를 켠 채로 PC 브라우저에서 `http://<파이IP>:8000` (파이 IP: `hostname -I`)

| 조작 | 방법 |
| --- | --- |
| 출발 / 정지 | Space 또는 버튼 |
| 녹화 | R (출발 중이고 선을 찾은 프레임만, 초당 10장 → `data/<날짜_시간>_lab/`) |
| 종료 | 터미널에서 Ctrl+C (모터 정지) |
| 안전 | 브라우저를 닫거나 응답이 1초 끊기면 정지, 선을 6프레임 연속 놓치면 정지 |

## 3. 화면 보는 법

- 하늘색 줄: 자르는 위치 (roi_top) · 초록 띠: 선을 찾는 곳 (look) · 빨간 점: 찾은 선
- 주황 화살표: 조향 방향 · 오른쪽 흑백 화면: mask (선만 하얗게 보여야 함)
- 맨 위 글자: `상태 x 오차 조향 명령` — 상태 RUN(주행) / HOLD(잠깐 놓침) / LOST(정지) / STOP

## 4. 트랙에서 슬라이더 조절 (한 번에 하나씩)

| 슬라이더 | 처음 | 이럴 때 |
| --- | --- | --- |
| thresh | 80 | mask에 바닥 얼룩 → ↓, 선이 끊김 → ↑ |
| roi_top | 0.45 | 벽 · 바깥 물건이 mask에 보임 → ↑ |
| look | 0.6 | 커브를 늦게 돎 → ↓, 너무 일찍 안쪽으로 → ↑ |
| kp | 1.0 | 커브에서 바깥으로 나감 → ↑, 직선에서 지그재그 → ↓ |
| kd | 0.5 | 흔들림이 계속 → ↑, 덜컥거림 → ↓ |
| speed | 0.35 | 처음엔 0.25~0.35, 바퀴가 안 돌면 ↑ |

잘 되는 값은 적어 두기 → 텀 프로젝트 `rc-car/pi/config.py`에 사용.

## 5. 안 될 때

| 증상 | 확인 |
| --- | --- |
| `[모터] 아두이노를 찾을 수 없음` | USB 연결 (`ls /dev/ttyACM*`), PC 시리얼 모니터 · test_motor.py 끄기 |
| `Permission denied: /dev/ttyACM0` | `sudo usermod -aG dialout $USER` → 로그아웃 후 다시 로그인 |
| 카메라 `Device or resource busy` | 카메라 쓰는 다른 프로그램(rpicam-hello, 다른 cv_lab.py) 끄기 |
| `ModuleNotFoundError` | `source venv/bin/activate` 했는지, venv를 `--system-site-packages`로 만들었는지 |
| 선이 오른쪽인데 왼쪽으로 꺾음 | `cv_lab.py` 위쪽 `ROTATE_180 = True` |
| mask가 전부 흰색 / 검은색 | thresh 조절, 조명 |
| 화살표는 맞는데 바퀴가 안 돎 | 배터리, speed, 스케치의 MIN_PWM |
