"""
공용 전처리 모듈 — PC(학습)와 라즈베리파이(추론)에서 반드시 같은 파일을 사용할 것.
입력: OpenCV BGR 이미지 (uint8, H x W x 3)
출력: PilotNet 입력 (float32, 66 x 200 x 3, YUV, 0~1)
"""
import cv2
import numpy as np

IMG_H, IMG_W = 66, 200
CROP_TOP = 0.40      # 이미지 위쪽 40% 제거 (카메라 각도에 맞춰 조정, 차선이 잘리지 않게)
CROP_BOTTOM = 0.0    # 차체 앞부분이 찍히면 아래쪽도 잘라냄 (예: 0.05)


def crop(bgr):
    h = bgr.shape[0]
    top = int(h * CROP_TOP)
    bottom = h - int(h * CROP_BOTTOM)
    return bgr[top:bottom, :, :]


def preprocess(bgr):
    img = crop(bgr)
    img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    return img.astype(np.float32) / 255.0
