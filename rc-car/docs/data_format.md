# 학습 데이터 · 모델 저장 규칙 v1

수집 코드(`pi/collect.py`)가 이 형식으로 저장하고, 학습 코드(`train/train_pilotnet.py`)가 이 형식을 읽는다.
바꿀 때는 `docs/protocol.md`의 변경 절차를 따른다. 기준: 실습 11(cv_lab) C5 `save_sample`.

## 폴더 구조

```
data/
  20261120_143000_auto/        # YYYYMMDD_HHMMSS_<auto|manual|recovery>
    images/
      000001.jpg
      000002.jpg
    labels.csv
    info.json
```

- 주행 1회 = 폴더 1개. 학습 시 검증은 이 폴더 단위로 분리한다.
- 데이터는 git에 올리지 않고 구글 드라이브 `rc_data/`에 둔다.

## labels.csv

```
image,steering,throttle,time
images/000001.jpg,0.25,0.35,1763255700.12
```

| 열 | 내용 |
| --- | --- |
| `image` | 폴더 기준 상대 경로 (`images/` + 6자리 번호) |
| `steering` | **정답 조향** −1 ~ +1. 노이즈를 섞어 주행했어도 노이즈를 뺀 값(OpenCV 또는 사람이 낸 값) |
| `throttle` | 그때의 speed 0 ~ 1 |
| `time` | `time.time()` (초) |

- 머리줄은 처음 한 번만, 이후 추가 모드(`'a'`)로 한 줄씩 기록
- 사진은 카메라 원본(320×240 BGR)을 JPEG 품질 90으로 저장. 전처리는 학습 때 적용

## 언제 저장하나

- 초당 10장
- 출발 상태이면서 선을 찾은 프레임만 (선을 놓쳐 HOLD 중인 프레임은 제외)

## info.json

녹화 시작 시 한 번 기록한다.

```json
{
  "start": "2026-11-20 14:30:00",
  "mode": "auto",
  "settings": {"thresh": 80, "roi_top": 0.45, "look": 0.6, "kp": 1.0, "kd": 0.5, "speed": 0.35},
  "noise": 0.0,
  "light": "형광등",
  "memo": ""
}
```

## 모델

- 드라이브 `models/vN/pilotnet.tflite` (+ `pilotnet.keras`), 버전마다 새 폴더
- 학습에 쓴 데이터 폴더와 주요 설정은 `docs/experiments.md`에 기록
- 파이에서는 `pi/config.py`의 `MODEL_PATH` 위치에 복사해서 사용

> 할 일: `train/train_pilotnet.py`의 `load_runs`는 아직 예전 형식(`filename` 열)을 읽는다. 이 형식(`image` 열)에 맞게 수정 필요.
