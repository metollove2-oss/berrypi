# rc-car — CNN 조향 기반 자율주행 RC카

임베디드시스템설계 팀 프로젝트. 파이카메라 영상을 CNN(PilotNet)으로 해석해 조향하고, 초음파 센서로 장애물 앞에서 정지한다.

## 구성

| 폴더 | 내용 | 실행 위치 | 담당 |
| --- | --- | --- | --- |
| `arduino/firmware/` | 시리얼 명령 수신, 모터(L293D), 초음파, LCD, 가변저항 | Arduino Uno | 팀원 2 |
| `pi/` | 주행, 데이터 수집, OpenCV 차선 검출, 시리얼 통신 | Raspberry Pi 5 | 팀원 1 |
| `common/` | PC와 파이가 같이 쓰는 전처리 | 둘 다 | 공통 (수정 시 서로 공유) |
| `train/` | PilotNet 학습, TFLite 변환 | PC / Colab | 팀원 1 |
| `tools/` | 보조 스크립트 | PC | - |
| `docs/` | 설계, 통신 규칙, 실험 기록 | - | 공통 |

## 데이터와 모델

이미지 데이터와 모델 파일은 git에 올리지 않고 구글 드라이브에 둔다.

- 데이터: (드라이브 링크) `rc_data/run_XX/labels.csv + 이미지`
- 모델: (드라이브 링크) `models/vN/pilotnet.tflite` — 버전별로 저장하고 `docs/experiments.md`에 기록

## 실행

**라즈베리파이**
```bash
git clone https://github.com/<계정>/rc-car.git && cd rc-car
python3 -m venv --system-site-packages .venv && source .venv/bin/activate
pip install -r pi/requirements.txt
python -m pi.collect          # 데이터 수집
python -m pi.drive --mode cnn # 주행 (opencv | cnn)
```

**학습 (Colab)** — `train/train_pilotnet.py` 상단 설명 참고

**아두이노** — `arduino/firmware/firmware.ino`를 Arduino IDE로 열어 업로드

## 작업 규칙

1. 작업 전 `git pull`, 작업 후 바로 commit + push
2. 커밋 메시지는 무엇을 바꿨는지 한 줄로 (예: `초음파 정지 거리 200mm로 변경`)
3. `common/`와 `docs/protocol.md`를 바꿀 때는 미리 서로 알린다
4. 트랙에서 바꾼 튜닝 값(`pi/config.py`)도 그 자리에서 커밋한다
