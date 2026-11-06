"""
PilotNet 조향 모델 학습 스크립트 (Colab / PC 용)

[데이터 폴더 구조]  — 라즈베리파이 수집 스크립트가 이 형식으로 저장
  data/
    run_01/
      labels.csv        # 열: filename,steering   (steering: -1=최대 좌, 0=직진, +1=최대 우)
      img_000001.jpg
      ...
    run_02/
    ...
  주행 1회(트랙 몇 바퀴)를 run 폴더 하나로 저장한다. 검증은 run 단위로 분리한다.

[Colab 실행 예]
  from google.colab import drive; drive.mount('/content/drive')
  !git clone https://github.com/<계정>/rc-car.git
  !python rc-car/train/train_pilotnet.py --data /content/drive/MyDrive/rc_data --val-runs run_05 --out /content/drive/MyDrive/rc_model

[이어서 학습(파인튜닝) 예]
  !python train_pilotnet.py --data ... --init /content/drive/MyDrive/rc_model/pilotnet.keras --lr 1e-4

[출력]
  pilotnet.keras        학습된 모델 (재학습/파인튜닝용)
  pilotnet.tflite       라즈베리파이 추론용
  steering_hist.png     학습 데이터 조향값 분포 (균형화 전/후)
  loss.png              학습/검증 손실 곡선
  val_prediction.png    검증 주행에서 정답 vs 예측 조향값
"""
import argparse
import glob
import os
import random

import cv2
import numpy as np
import pandas as pd
import tensorflow as tf
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common.preprocess import IMG_H, IMG_W, preprocess   # 저장소 최상위의 common/ 사용

# ---------------- 증강 설정 ----------------
SHIFT_MAX = 0.10     # 좌우 평행 이동 최대 비율 (이미지 폭 대비)
SHIFT_GAIN = 2.0     # 이동 비율당 조향 보정량 (0.1 이동 → +0.2). 트랙에서 튜닝
P_FLIP, P_SHADOW, P_SHIFT = 0.5, 0.3, 0.5


# ---------------- 데이터 로드 ----------------
def load_runs(data_dir):
    frames = []
    for run_dir in sorted(glob.glob(os.path.join(data_dir, "*"))):
        csv_path = os.path.join(run_dir, "labels.csv")
        if not os.path.isfile(csv_path):
            continue
        df = pd.read_csv(csv_path)
        df["path"] = df["filename"].apply(lambda f: os.path.join(run_dir, f))
        df["run"] = os.path.basename(run_dir)
        frames.append(df[["path", "steering", "run"]])
    if not frames:
        raise SystemExit(f"labels.csv가 있는 run 폴더를 찾지 못함: {data_dir}")
    df = pd.concat(frames, ignore_index=True)
    df = df[df["path"].apply(os.path.isfile)].reset_index(drop=True)
    return df


def smooth_labels(df, window):
    """OpenCV 라벨의 흔들림을 run별 이동평균으로 완화"""
    if window <= 1:
        return df
    df = df.copy()
    df["steering"] = (
        df.groupby("run")["steering"]
        .transform(lambda s: s.rolling(window, center=True, min_periods=1).mean())
    )
    return df


def balance(df, keep_straight, thr=0.05, seed=0):
    """직진(|s| < thr) 샘플을 keep_straight 비율만 남겨 '항상 직진' 학습을 방지"""
    straight = df[df["steering"].abs() < thr].sample(frac=keep_straight, random_state=seed)
    turn = df[df["steering"].abs() >= thr]
    return pd.concat([straight, turn]).sample(frac=1, random_state=seed).reset_index(drop=True)


# ---------------- 증강 ----------------
def augment(img, s):
    h, w = img.shape[:2]
    # 1) 좌우 반전: 이미지와 조향 부호를 함께 뒤집음
    if random.random() < P_FLIP:
        img = cv2.flip(img, 1)
        s = -s
    # 2) 밝기 변화
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[..., 2] *= random.uniform(0.6, 1.3)
    img = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)
    # 3) 무작위 그림자
    if random.random() < P_SHADOW:
        x1, x2 = random.randint(0, w - 1), random.randint(0, w - 1)
        x3, x4 = random.randint(0, w - 1), random.randint(0, w - 1)
        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [np.array([[x1, 0], [x2, 0], [x3, h - 1], [x4, h - 1]], np.int32)], 1)
        img = img.astype(np.float32)
        img[mask == 1] *= random.uniform(0.5, 0.8)
        img = img.astype(np.uint8)
    # 4) 좌우 평행 이동 + 조향 보정 (차가 차선에서 벗어난 상황을 흉내 = 복구 데이터 효과)
    #    내용이 오른쪽으로 이동(frac>0) → 차가 상대적으로 왼쪽에 있음 → 오른쪽(+)으로 더 꺾어야 함
    if random.random() < P_SHIFT:
        frac = random.uniform(-SHIFT_MAX, SHIFT_MAX)
        M = np.float32([[1, 0, frac * w], [0, 1, 0]])
        img = cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REPLICATE)
        s += frac * SHIFT_GAIN
    return img, s


# ---------------- 배치 생성기 ----------------
class DriveSequence(tf.keras.utils.Sequence):
    def __init__(self, df, batch_size, augment_on, shuffle, **kwargs):
        super().__init__(**kwargs)
        self.df = df.reset_index(drop=True)
        self.batch_size = batch_size
        self.augment_on = augment_on
        self.shuffle = shuffle
        self.idx = np.arange(len(self.df))
        self.on_epoch_end()

    def __len__(self):
        return int(np.ceil(len(self.df) / self.batch_size))

    def on_epoch_end(self):
        if self.shuffle:
            np.random.shuffle(self.idx)

    def __getitem__(self, i):
        ids = self.idx[i * self.batch_size:(i + 1) * self.batch_size]
        X = np.empty((len(ids), IMG_H, IMG_W, 3), np.float32)
        y = np.empty((len(ids),), np.float32)
        for k, j in enumerate(ids):
            row = self.df.iloc[j]
            img = cv2.imread(row["path"])
            s = float(row["steering"])
            if self.augment_on:
                img, s = augment(img, s)
            X[k] = preprocess(img)
            y[k] = np.clip(s, -1.0, 1.0)
        return X, y


# ---------------- 모델 (PilotNet) ----------------
def build_pilotnet(dropout=0.3):
    L = tf.keras.layers
    return tf.keras.Sequential([
        L.Input(shape=(IMG_H, IMG_W, 3)),
        L.Conv2D(24, 5, strides=2, activation="relu"),   # 31x98x24
        L.Conv2D(36, 5, strides=2, activation="relu"),   # 14x47x36
        L.Conv2D(48, 5, strides=2, activation="relu"),   # 5x22x48
        L.Conv2D(64, 3, activation="relu"),              # 3x20x64
        L.Conv2D(64, 3, activation="relu"),              # 1x18x64
        L.Flatten(),                                     # 1152
        L.Dropout(dropout),
        L.Dense(100, activation="relu"),
        L.Dense(50, activation="relu"),
        L.Dense(10, activation="relu"),
        L.Dense(1, activation="tanh"),                   # 조향값 -1 ~ +1
    ], name="pilotnet")


# ---------------- 시각화 ----------------
def save_hist(raw, balanced, path):
    plt.figure(figsize=(8, 3))
    bins = np.linspace(-1, 1, 41)
    plt.hist(raw, bins=bins, alpha=0.5, label="raw")
    plt.hist(balanced, bins=bins, alpha=0.7, label="balanced")
    plt.xlabel("steering")
    plt.ylabel("count")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=120)
    plt.close()


def save_loss(history, path):
    plt.figure(figsize=(6, 3))
    plt.plot(history.history["loss"], label="train")
    plt.plot(history.history["val_loss"], label="val")
    plt.xlabel("epoch")
    plt.ylabel("MSE")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=120)
    plt.close()


def save_val_prediction(model, val_df, path):
    seq = DriveSequence(val_df, 128, augment_on=False, shuffle=False)
    pred = model.predict(seq, verbose=0).ravel()
    true = val_df["steering"].clip(-1, 1).values
    plt.figure(figsize=(10, 3))
    plt.plot(true, label="label", linewidth=1)
    plt.plot(pred, label="prediction", linewidth=1)
    plt.xlabel("frame")
    plt.ylabel("steering")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=120)
    plt.close()
    return float(np.mean((pred - true) ** 2))


def export_tflite(model, path, quantize):
    conv = tf.lite.TFLiteConverter.from_keras_model(model)
    if quantize:
        conv.optimizations = [tf.lite.Optimize.DEFAULT]   # 가중치 int8 양자화 (크기↓, 속도↑)
    with open(path, "wb") as f:
        f.write(conv.convert())


# ---------------- 메인 ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="run 폴더들이 있는 상위 폴더")
    ap.add_argument("--out", default="model_out")
    ap.add_argument("--val-runs", nargs="*", default=None, help="검증용 run 이름 (없으면 마지막 run)")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--keep-straight", type=float, default=0.4, help="직진 샘플 유지 비율")
    ap.add_argument("--smooth", type=int, default=5, help="라벨 이동평균 창 크기 (1이면 끔)")
    ap.add_argument("--init", default=None, help="이어서 학습할 .keras 모델 경로 (파인튜닝)")
    ap.add_argument("--quant", action="store_true", help="TFLite 변환 시 양자화")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    tf.random.set_seed(args.seed)
    os.makedirs(args.out, exist_ok=True)

    # 1) 데이터 로드 및 run 단위 분할
    df = smooth_labels(load_runs(args.data), args.smooth)
    runs = sorted(df["run"].unique())
    val_runs = args.val_runs or [runs[-1]]
    if len(runs) < 2:
        raise SystemExit("run 폴더가 2개 이상 필요함 (학습용 + 검증용)")
    train_df = df[~df["run"].isin(val_runs)]
    val_df = df[df["run"].isin(val_runs)].reset_index(drop=True)

    # 2) 학습 데이터만 균형화
    train_bal = balance(train_df, args.keep_straight, seed=args.seed)
    save_hist(train_df["steering"], train_bal["steering"], os.path.join(args.out, "steering_hist.png"))
    print(f"runs: {runs} | val: {val_runs}")
    print(f"train {len(train_df)} -> balanced {len(train_bal)} | val {len(val_df)}")

    train_seq = DriveSequence(train_bal, args.batch, augment_on=True, shuffle=True)
    val_seq = DriveSequence(val_df, args.batch, augment_on=False, shuffle=False)

    # 3) 모델 생성 또는 불러오기
    if args.init:
        model = tf.keras.models.load_model(args.init)
        print(f"loaded {args.init} (fine-tuning)")
    else:
        model = build_pilotnet()
    model.compile(optimizer=tf.keras.optimizers.Adam(args.lr), loss="mse", metrics=["mae"])
    model.summary()

    # 4) 학습
    keras_path = os.path.join(args.out, "pilotnet.keras")
    callbacks = [
        tf.keras.callbacks.EarlyStopping(monitor="val_loss", patience=6, restore_best_weights=True),
        tf.keras.callbacks.ReduceLROnPlateau(monitor="val_loss", factor=0.5, patience=3),
        tf.keras.callbacks.ModelCheckpoint(keras_path, monitor="val_loss", save_best_only=True),
    ]
    history = model.fit(train_seq, validation_data=val_seq, epochs=args.epochs, callbacks=callbacks)

    # 5) 평가 및 저장
    save_loss(history, os.path.join(args.out, "loss.png"))
    val_mse = save_val_prediction(model, val_df, os.path.join(args.out, "val_prediction.png"))
    model.save(keras_path)
    tflite_path = os.path.join(args.out, "pilotnet.tflite")
    export_tflite(model, tflite_path, args.quant)
    print(f"val MSE = {val_mse:.4f}")
    print(f"saved: {keras_path}, {tflite_path}")


if __name__ == "__main__":
    main()
