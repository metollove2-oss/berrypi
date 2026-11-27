"""
OpenCV 선 따라가기 (임베디드시스템설계 11. cv_lab 기준, 베리파이 팀 완성본)

카메라 → [C1] make_mask → [C2] find_line_x → [C3] line_error → [C4] pd_steer
       → 커브 감속 → mix → to_command → 아두이노 (M,<좌>,<우>)
녹화 중이면 [C5] save_sample 로 사진 + labels.csv 저장 (학습 데이터)

선 검출 개선 (웹 체크박스로 켜고 끄기): ① 아래 연결 덩어리만 ② 선 폭 검사
③ 지난 위치에 가까운 선 ④ 급격한 점프 무시  → make_mask 다음, C3 전에 적용

실행 (라즈베리파이):
    source venv/bin/activate
    python3 cv_lab.py
    브라우저 http://<파이IP>:8000   (Space = 출발/정지, R = 녹화)

아두이노에는 강의의 motor_serial.ino (또는 motor_lab.ino) 가 올라가 있어야 함.
"""
import csv
import glob
import os
import threading
import time

import cv2
import numpy as np

# ===================== 설정 (필요하면 여기만 바꾸기) =====================
ROTATE_180 = False          # 카메라를 거꾸로 달았으면 True
WIDTH, HEIGHT = 320, 240    # 처리 해상도
SERIAL_PORT = None          # None 이면 /dev/ttyACM*, /dev/ttyUSB* 자동 검색
BAUD = 115200
SLOWDOWN = 0.4              # 커브 감속: speed × (1 − SLOWDOWN × |steer|)
LOST_LIMIT = 6              # 선을 연속으로 이만큼 놓치면 정지
RECORD_HZ = 10              # 녹화 주기 (초당 장수)
WEB_PORT = 8000
BAND = 0.16                 # C2 띠 높이 (mask 높이 대비)
BROWSER_TIMEOUT = 1.0       # 브라우저 응답이 이만큼 끊기면 정지 (초)

# 웹 슬라이더로 바꾸는 값 (시작값)
SETTINGS = {
    "thresh": 80,      # 이진화 기준 (0~255)
    "roi_top": 0.45,   # 위쪽 자르기 비율 (0~0.8)
    "look": 0.6,       # 띠 위치 (0.1~0.9, 작을수록 멀리)
    "kp": 1.0,
    "kd": 0.5,
    "speed": 0.35,     # 0~1
}
SLIDERS = [  # (이름, 최소, 최대, 간격)
    ("thresh", 0, 255, 1), ("roi_top", 0.0, 0.8, 0.01), ("look", 0.1, 0.9, 0.01),
    ("kp", 0.0, 3.0, 0.05), ("kd", 0.0, 2.0, 0.05), ("speed", 0.0, 1.0, 0.01),
    ("min_bottom", 0.5, 1.0, 0.02), ("max_w", 10, 160, 1), ("max_jump", 20, 160, 5),
]

# ---- 선 검출 개선 (웹 화면 체크박스로 켜고 끄기, 1 = 켬)
IMPROVE = {
    "bottom": 1,    # ① 화면 아래쪽까지 이어진 흰 덩어리만 선으로 봄 (떠 있는 물체 제거)
    "width": 1,     # ② 띠 안의 흰 구간 중 선 폭(MIN_W ~ max_w 픽셀)에 맞는 것만 후보
    "nearest": 1,   # ③ 후보가 여러 개면 지난 프레임 x에 가장 가까운 것 선택
    "jump": 1,      # ④ x가 max_jump 픽셀 넘게 갑자기 튀면 무시 (3프레임 연속이면 인정)
}
SWITCHES = [("bottom", "① 아래 연결"), ("width", "② 선 폭"), ("nearest", "③ 가까운 선"), ("jump", "④ 점프 무시")]
SETTINGS.update({
    "min_bottom": 0.9,  # ① 덩어리 아래 끝이 mask 높이의 이 비율보다 아래(차 바로 앞)까지 와야 함
    "max_w": 60,        # ② 선으로 인정할 최대 폭 (픽셀, 320 기준). 테이프가 굵으면 ↑
    "max_jump": 60,     # ④ 한 프레임에 이보다 많이 튀면 착각으로 봄 (픽셀)
})
# ---- 충돌 방지 (초음파, 웹 체크박스 / 슬라이더)
SAFETY = {
    "safety": 1,        # 충돌 방지 켬: 주의 구간 감속, 위험 구간 정지
    "sound": 1,         # 아두이노 경고음 켬
    "auto_resume": 1,   # 장애물이 치워지면 자동 재출발 (끄면 Space를 다시 눌러야 출발)
}
SAFETY_SWITCHES = [("safety", "충돌 방지"), ("sound", "경고음"), ("auto_resume", "자동 재출발")]
SLIDERS += [("caution_cm", 15, 60, 1), ("danger_cm", 8, 30, 1), ("emerg_cm", 4, 15, 1)]
SETTINGS.update({
    "caution_cm": 30,   # 이보다 가까우면 주의: 노란 띠 + 감속
    "danger_cm": 15,    # 이보다 가까우면 위험: 빨간 경고창 + 정지
    "emerg_cm": 8,      # 이보다 가까우면 아두이노가 직접 비상 정지
})
HYST_CM = 3             # 구간에서 나올 때 여유 (아두이노 HYST_MM 과 같게)
RESUME_HOLD = 0.5       # 위험 구간을 벗어난 상태가 이만큼 이어져야 재출발 (초)
MIN_SLOW = 0.3          # 주의 구간 가장 가까운 곳에서의 속도 비율
DIST_TIMEOUT = 1.0      # 거리 메시지가 이만큼 안 오면 센서 오류 (초)

# ---- 자동 데이터 수집 (웹 체크박스 / 슬라이더)
COLLECT = {
    "noise": 0,         # 자동 흔들기: 주행 조향에 일부러 잡음을 섞음 (정답은 잡음 없는 OpenCV 조향으로 저장)
    "confident": 1,     # 확실할 때만 저장: 후보 1개 · 점프 없음 · 놓침 없음인 프레임만 녹화
}
COLLECT_SWITCHES = [("noise", "자동 흔들기"), ("confident", "확실할 때만 저장")]
SLIDERS += [("noise_amp", 0.0, 0.6, 0.05)]
SETTINGS.update({
    "noise_amp": 0.3,   # 흔들기 세기 (조향 값 기준). 너무 크면 선을 놓침 → 0.2~0.4 권장
})
NOISE_ON_RATIO = 0.5    # 흔드는 구간 비율 (나머지는 잡음 0 → 차가 스스로 선으로 돌아오는 장면)
NOISE_HOLD = (0.4, 0.9) # 잡음 하나를 유지하는 시간 범위 (초)

MIN_W = 3               # ② 선으로 인정할 최소 폭 (픽셀)
MIN_AREA = 30           # ① 이보다 작은 덩어리는 잡티로 버림 (픽셀 수)
JUMP_ACCEPT = 3         # ④ 같은 방향으로 이만큼 연속 튀면 진짜 선으로 인정


# =====================================================================
# [C1] 사진 → 선만 흰색인 mask
def make_mask(bgr, roi_top, thresh):
    h = bgr.shape[0]
    y0 = int(h * roi_top)                                   # 위쪽(벽, 먼 곳) 자르기
    roi = bgr[y0:]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)            # 흑백
    gray = cv2.GaussianBlur(gray, (5, 5), 0)                # 작은 얼룩 약하게
    _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)  # 어두운 선 → 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 잡티 제거
    return mask, y0


# [C2] mask → 띠 하나에서 선의 x (없으면 None)
def find_line_x(mask, look):
    h = mask.shape[0]
    b0 = int(np.clip(h * look - h * BAND / 2, 0, h - 2))   # 띠 위 줄
    b1 = int(np.clip(b0 + h * BAND, b0 + 2, h))             # 띠 아래 줄
    counts = (mask[b0:b1] > 0).sum(axis=0)                  # 열마다 흰 점 수
    cols = np.where(counts >= max(2, (b1 - b0) // 4))[0]    # 띠 높이 1/4 이상 = 선
    return None if len(cols) == 0 else float(cols.mean())


# [C3] 선 위치 → 오차 −1 ~ +1 (선이 왼쪽이면 음수)
def line_error(x, width):
    return (x - width / 2) / (width / 2)


# [C4] 오차 → 조향 (PD 제어) −1 ~ +1
def pd_steer(error, prev_error, dt, kp, kd):
    change = (error - prev_error) / dt * 0.1 if dt > 0 else 0.0   # 0.1초 동안의 변화
    return max(-1.0, min(1.0, kp * error + kd * change))


# [C5] 사진 한 장 + labels.csv 한 줄
def save_sample(folder, index, frame, steering, throttle):
    os.makedirs(os.path.join(folder, "images"), exist_ok=True)
    name = f"images/{index:06d}.jpg"
    cv2.imwrite(os.path.join(folder, name), frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    csv_path = os.path.join(folder, "labels.csv")
    new = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["image", "steering", "throttle", "time"])
        w.writerow([name, steering, throttle, time.time()])
    return name


# =====================================================================
# 선 검출 개선 ① ~ ③  (④ 점프 무시는 Driver.step 에서 지난 x와 비교)
def band_rows(h, look):
    """C2와 같은 띠 위/아래 줄"""
    b0 = int(np.clip(h * look - h * BAND / 2, 0, h - 2))
    b1 = int(np.clip(b0 + h * BAND, b0 + 2, h))
    return b0, b1


# ① 화면 아래쪽까지 이어진 덩어리만 남기기
def keep_bottom(mask, min_bottom, min_area=MIN_AREA):
    """진짜 선은 차 바로 앞(mask 아래쪽)부터 이어져 있음 → 위나 옆에 떠 있는 덩어리는 버림"""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    h = mask.shape[0]
    good = np.zeros(n, bool)
    for i in range(1, n):
        y, hh, area = stats[i, cv2.CC_STAT_TOP], stats[i, cv2.CC_STAT_HEIGHT], stats[i, cv2.CC_STAT_AREA]
        good[i] = area >= min_area and (y + hh) >= h * min_bottom
    return np.where(good[labels], 255, 0).astype(np.uint8)


# 띠 안에서 선으로 보이는 열들을 이어진 구간(segment)으로 나누기
def find_segments(mask, look):
    h = mask.shape[0]
    b0, b1 = band_rows(h, look)
    counts = (mask[b0:b1] > 0).sum(axis=0)
    on = (counts >= max(2, (b1 - b0) // 4)).astype(np.int8)
    edges = np.diff(np.concatenate([[0], on, [0]]))
    starts, ends = np.where(edges == 1)[0], np.where(edges == -1)[0]
    return list(zip(starts.tolist(), ends.tolist()))      # [(시작 x, 끝 x(포함 안 함)), ...]


# ② 선 폭 검사 + ③ 가까운 선 선택 → 선의 x (없으면 None)
def detect_line(mask, look, prev_x, use_width, use_nearest, max_w, min_w=MIN_W):
    segs = find_segments(mask, look)
    cands = [s for s in segs if min_w <= s[1] - s[0] <= max_w] if use_width else segs
    if not cands:
        return None, segs, None
    if not use_nearest:                                   # C2와 같은 방식: 후보 열 전체의 평균
        cols = np.concatenate([np.arange(a, b) for a, b in cands])
        return float(cols.mean()), segs, None
    ref = prev_x if prev_x is not None else mask.shape[1] / 2   # 지난 x (없으면 화면 가운데)
    best = min(cands, key=lambda s: abs((s[0] + s[1] - 1) / 2 - ref))
    return (best[0] + best[1] - 1) / 2, segs, best


# =====================================================================
# 충돌 방지: 거리 → 구간, 감속 비율
ZONE_RANK = {"SAFE": 0, "CAUTION": 1, "DANGER": 2, "EMERG": 3, "NOSENSOR": 0}


def safety_zone(dist_mm, prev_zone, caution_cm, danger_cm, emerg_cm, hyst_cm=HYST_CM):
    """들어갈 때는 기준 거리, 나올 때는 기준 + hyst_cm (경계에서 깜빡임 방지)"""
    if dist_mm is None or dist_mm < 0:
        return "NOSENSOR"
    d, p = dist_mm / 10.0, ZONE_RANK.get(prev_zone, 0)
    if d < emerg_cm:
        return "EMERG"
    if d < danger_cm or (d < danger_cm + hyst_cm and p >= 2):
        return "DANGER"
    if d < caution_cm or (d < caution_cm + hyst_cm and p >= 1):
        return "CAUTION"
    return "SAFE"


def safety_speed_scale(dist_mm, caution_cm, danger_cm, min_slow=MIN_SLOW):
    """주의 구간에서 거리에 비례한 속도 비율: caution_cm → 1.0, danger_cm → min_slow"""
    ratio = (dist_mm / 10.0 - danger_cm) / max(1e-6, caution_cm - danger_cm)
    return min_slow + (1 - min_slow) * float(np.clip(ratio, 0.0, 1.0))


# =====================================================================
# 모터 (motor_lab 에서 만든 것과 같음)
def mix(speed, steer):
    left, right = speed * (1 + steer), speed * (1 - steer)
    m = max(abs(left), abs(right))
    if m > 1:                                  # 1을 넘으면 비율 유지하며 줄이기
        left, right = left / m, right / m
    return left, right


def to_command(left, right):
    left = max(-1.0, min(1.0, left))
    right = max(-1.0, min(1.0, right))
    return f"M,{int(round(left * 255))},{int(round(right * 255))}\n"


class MotorLink:
    """아두이노 시리얼 연결. 못 찾으면 모터 없이(화면만) 동작"""

    def __init__(self, port=SERIAL_PORT, baud=BAUD):
        self.ser = None
        try:
            import serial
        except ImportError:
            print("[모터] pyserial 없음 → sudo apt install python3-serial")
            return
        ports = [port] if port else sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        for p in ports:
            try:
                self.ser = serial.Serial(p, baud, timeout=0.05)
                time.sleep(2)                     # 포트를 열면 아두이노가 리셋됨
                self.ser.reset_input_buffer()
                print(f"[모터] 아두이노 연결: {p}")
                threading.Thread(target=self._reader, daemon=True).start()
                return
            except Exception as e:
                print(f"[모터] {p} 열기 실패: {e}")
        print("[모터] 아두이노를 찾을 수 없음 → 화면만 동작 (USB 연결, 시리얼 모니터 닫았는지 확인)")

    # ---- 아두이노 → 파이 메시지 받기 (충돌 방지)
    dist_mm = None        # 마지막 거리 (mm). -1 = 센서 응답 없음, None = 아직 못 받음
    dist_time = 0.0       # 마지막 R 메시지 받은 시각
    emergency = False     # 아두이노 비상 정지 중 (E,1)

    def _reader(self):
        while self.ser:
            try:
                raw = self.ser.readline()
            except Exception:
                time.sleep(0.1)
                continue
            text = raw.decode(errors="ignore").strip()
            if text.startswith("R,"):
                try:
                    self.dist_mm, self.dist_time = int(text[2:]), time.time()
                except ValueError:
                    pass
            elif text == "E,1":
                self.emergency = True
            elif text == "E,0":
                self.emergency = False

    @property
    def ok(self):
        return self.ser is not None

    def send(self, text):
        if self.ser:
            try:
                self.ser.write(text.encode())
            except Exception as e:
                print(f"[모터] 전송 실패: {e}")

    def stop(self):
        self.send("M,0,0\n")

    def close(self):
        self.stop()
        if self.ser:
            time.sleep(0.05)
            self.ser.close()


# =====================================================================
# 카메라
class Camera:
    def __init__(self):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        cfg = None
        try:
            # 센서 전체 화각을 쓰는 모드 중 가장 작은 해상도 선택 (화면이 확대되어 좁게 보이지 않게)
            modes = self.cam.sensor_modes
            full = max(m["crop_limits"][2] * m["crop_limits"][3] for m in modes)
            mode = min((m for m in modes if m["crop_limits"][2] * m["crop_limits"][3] == full),
                       key=lambda m: m["size"][0] * m["size"][1])
            cfg = self.cam.create_video_configuration(
                main={"size": (WIDTH, HEIGHT), "format": "RGB888"}, raw={"size": mode["size"]})
            print(f"[카메라] 센서 모드 {mode['size']}")
        except Exception as e:
            print(f"[카메라] 센서 모드 선택 실패, 기본 설정 사용: {e}")
            cfg = self.cam.create_video_configuration(main={"size": (WIDTH, HEIGHT), "format": "RGB888"})
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(0.5)

    def read(self):
        frame = self.cam.capture_array()   # picamera2 의 RGB888 = OpenCV BGR 순서
        if frame.shape[2] == 4:
            frame = frame[:, :, :3]
        if ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return np.ascontiguousarray(frame)

    def close(self):
        self.cam.stop()


# =====================================================================
# 주행 반복
class Driver:
    def __init__(self, camera, link):
        self.camera, self.link = camera, link
        self.lock = threading.Lock()
        self.running = False
        self.recording = False
        self.last_ping = time.time()
        self.jpeg = None
        self.status = {}
        self.prev_error = 0.0
        self.prev_t = None
        self.steer = 0.0
        self.lost = 0
        self.rec_folder = None
        self.rec_index = 0
        self.rec_last = 0.0
        self.quit = False
        self.prev_x = None      # 지난 프레임에서 인정한 선 x (③ ④ 기준)
        self.jump_count = 0     # ④ 연속으로 튄 횟수
        self.miss = 0           # 선을 연속으로 못 찾은 횟수 (정지 중에도 셈)
        self.zone = "SAFE"      # 충돌 방지 구간
        self.obst = False       # 장애물 때문에 정지 중
        self.clear_since = None # 위험 구간을 벗어난 시각 (재출발 판단)
        self.ard_sent = {}      # 아두이노에 보낸 ARM / MUTE / TH 마지막 값
        self.ard_sent_t = 0.0
        self.noise = 0.0        # 자동 흔들기: 지금 섞고 있는 잡음
        self.noise_until = 0.0  # 이 시각에 새 잡음을 고름
        self.rec_skipped = 0    # '확실할 때만 저장'으로 건너뛴 프레임 수
        self.last_jump_t = 0.0  # ④ 점프를 마지막으로 감지한 시각
        self.drive_steer = 0.0  # 실제로 보낸 조향 (흔들기 포함)
        self.rng = np.random.default_rng()

    def next_noise(self, now, amp):
        """잡음을 일정 시간 유지하다 바꿈. 절반은 0 (차가 스스로 돌아오는 구간)"""
        if now >= self.noise_until:
            on = self.rng.random() < NOISE_ON_RATIO
            self.noise = float(self.rng.uniform(-amp, amp)) if on else 0.0
            self.noise_until = now + float(self.rng.uniform(*NOISE_HOLD))
        return self.noise

    # ---- 웹에서 부르는 것
    def toggle_run(self):
        with self.lock:
            self.running = not self.running
            self.prev_t, self.lost = None, 0
            self.obst, self.clear_since = False, None
            if not self.running:
                self.link.stop()
        return self.running

    def toggle_record(self):
        with self.lock:
            self.recording = not self.recording
            if self.recording:
                base = os.path.join("data", time.strftime("%Y%m%d_%H%M%S") + "_lab")
                self.rec_folder, k = base, 1
                while os.path.exists(self.rec_folder):           # 1초 안에 다시 눌러도 덮어쓰지 않게
                    k += 1
                    self.rec_folder = f"{base}{k}"
                self.rec_index, self.rec_skipped = 0, 0
                os.makedirs(self.rec_folder, exist_ok=True)
                with open(os.path.join(self.rec_folder, "info.json"), "w") as f:
                    import json
                    json.dump({"start": time.strftime("%Y-%m-%d %H:%M:%S"), "settings": dict(SETTINGS),
                               "improve": dict(IMPROVE),
                               "safety": dict(SAFETY), "collect": dict(COLLECT)}, f, ensure_ascii=False, indent=2)
                print(f"[녹화] 시작: {self.rec_folder}")
            else:
                print(f"[녹화] 정지: {self.rec_index}장")
        return self.recording

    def ping(self):
        self.last_ping = time.time()

    # ---- 한 프레임 처리
    def step(self, frame):
        s, imp = dict(SETTINGS), dict(IMPROVE)
        raw_mask, y0 = make_mask(frame, s["roi_top"], int(s["thresh"]))
        mask = keep_bottom(raw_mask, s["min_bottom"]) if imp["bottom"] else raw_mask          # ①
        x, segs, best = detect_line(mask, s["look"], self.prev_x,
                                    imp["width"], imp["nearest"], s["max_w"])                 # ② ③
        reason = "ok" if x is not None else ("width" if segs else "none")
        n_valid = (sum(1 for a, b in segs if MIN_W <= b - a <= s["max_w"]) if imp["width"] else len(segs))
        if x is not None and imp["jump"] and self.prev_x is not None \
                and abs(x - self.prev_x) > s["max_jump"]:                                     # ④
            self.jump_count += 1
            self.last_jump_t = time.time()
            if self.jump_count < JUMP_ACCEPT:
                x, reason = None, "jump"
            else:
                self.jump_count = 0          # 계속 그 자리에 있으면 진짜 선으로 인정
        elif x is not None:
            self.jump_count = 0
        if x is not None:
            self.prev_x, self.miss = x, 0
        else:
            self.miss += 1
            if self.miss >= LOST_LIMIT:
                self.prev_x = None           # 오래 놓치면 기준 x 초기화 (다시 가운데 기준)
        now = time.time()

        if self.running and now - self.last_ping > BROWSER_TIMEOUT:
            print("[안전] 브라우저 응답 없음 → 정지")
            self.running = False

        # ---- 충돌 방지: 거리 → 구간 → 감속 / 정지
        dist = self.link.dist_mm
        if dist is not None and now - self.link.dist_time > DIST_TIMEOUT:
            dist = -1                                     # 거리 메시지 끊김 = 센서 오류
        self.zone = safety_zone(dist, self.zone, s["caution_cm"], s["danger_cm"], s["emerg_cm"])
        safety_on = bool(SAFETY["safety"]) and self.link.ok
        speed_scale = 1.0
        if self.running and safety_on:
            blocked = self.zone in ("DANGER", "EMERG", "NOSENSOR") or self.link.emergency
            if blocked:
                self.obst, self.clear_since = True, None
            elif self.obst:                                # 위험 구간을 벗어남 → 잠시 기다렸다 재출발
                self.clear_since = self.clear_since or now
                if now - self.clear_since >= RESUME_HOLD:
                    if SAFETY["auto_resume"]:
                        self.obst, self.clear_since = False, None
                    else:
                        self.running, self.obst = False, False     # 수동 재출발: Space 다시 누르기
            if not self.obst and self.zone == "CAUTION":
                speed_scale = safety_speed_scale(dist, s["caution_cm"], s["danger_cm"])
        else:
            self.obst, self.clear_since = False, None

        state, err, cmd = "STOP", None, "M,0,0\n"
        if self.running:
            if x is not None:
                err = line_error(x, frame.shape[1])
                dt = 0.0 if self.prev_t is None else now - self.prev_t
                prev = err if self.prev_t is None else self.prev_error
                self.steer = pd_steer(err, prev, dt, s["kp"], s["kd"])
                self.prev_error, self.prev_t, self.lost = err, now, 0
                state = "RUN"
            else:
                self.lost += 1
                state = "HOLD" if self.lost < LOST_LIMIT else "LOST"
            if self.obst:
                state = "OBST"                             # 장애물 정지 (명령 M,0,0, 녹화 안 함)
            self.drive_steer = self.steer
            if state in ("RUN", "HOLD"):
                # 자동 흔들기: 선을 찾았고 감속 중이 아닐 때만. 실제 조향 = 정답 조향 + 잡음
                if COLLECT["noise"] and state == "RUN" and speed_scale >= 1.0:
                    self.drive_steer = float(np.clip(self.steer + self.next_noise(now, s["noise_amp"]), -1, 1))
                else:
                    self.noise, self.noise_until = 0.0, 0.0
                speed = s["speed"] * speed_scale * (1 - SLOWDOWN * abs(self.drive_steer))
                cmd = to_command(*mix(speed, self.drive_steer))
            if state == "RUN" and self.recording and now - self.rec_last >= 1.0 / RECORD_HZ:
                sure = n_valid == 1 and now - self.last_jump_t > 0.3 and self.lost == 0
                if COLLECT["confident"] and not sure:
                    self.rec_skipped += 1                  # 확실하지 않은 프레임은 건너뜀
                else:
                    self.rec_index += 1                    # 정답 = 잡음 없는 OpenCV 조향
                    save_sample(self.rec_folder, self.rec_index, frame, round(self.steer, 4),
                                round(s["speed"] * speed_scale, 3))
                self.rec_last = now
        self.link.send(cmd)                       # 매 프레임 전송 = 하트비트
        self.sync_arduino(s, safety_on, now)

        self.status = {"state": state, "x": x, "err": err, "steer": self.steer if state != "STOP" else 0.0,
                       "cmd": cmd.strip(), "lost": self.lost, "recording": self.recording,
                       "rec_count": self.rec_index, "running": self.running, "motor": self.link.ok,
                       "reason": reason, "cands": len(segs), "improve": imp,
                       "dist_cm": None if dist is None or dist < 0 else round(dist / 10.0, 1),
                       "zone": self.zone, "emergency": self.link.emergency, "safety": safety_on,
                       "speed_scale": round(speed_scale, 2),
                       "noise": round(self.noise, 2) if state == "RUN" else 0.0,
                       "drive_steer": self.drive_steer if state in ("RUN", "HOLD") else 0.0,
                       "rec_skipped": self.rec_skipped, "n_valid": n_valid}
        self.jpeg = self.draw(frame, raw_mask, mask, y0, x, s, segs, best)

    def sync_arduino(self, s, safety_on, now):
        """경고음 조건(ARM) · 음소거(MUTE) · 구간 거리(TH)를 바뀔 때 + 1초마다 아두이노에 보냄"""
        want = {
            "ARM": f"ARM,{1 if (self.running and safety_on) else 0}\n",
            "MUTE": f"MUTE,{0 if SAFETY['sound'] else 1}\n",
            "TH": f"TH,{int(s['caution_cm'] * 10)},{int(s['danger_cm'] * 10)},{int(s['emerg_cm'] * 10)}\n",
        }
        resend = now - self.ard_sent_t > 1.0          # 아두이노가 리셋돼도 다시 맞춰지게
        for k, v in want.items():
            if resend or self.ard_sent.get(k) != v:
                self.link.send(v)
                self.ard_sent[k] = v
        if resend:
            self.ard_sent_t = now

    def draw(self, frame, raw_mask, mask, y0, x, s, segs, best):
        view = frame.copy()
        h, w = view.shape[:2]
        mh = mask.shape[0]
        cv2.line(view, (0, y0), (w, y0), (255, 255, 0), 1)                    # 하늘색: 자르는 위치
        b0 = int(np.clip(mh * s["look"] - mh * BAND / 2, 0, mh - 2))
        b1 = int(np.clip(b0 + mh * BAND, b0 + 2, mh))
        cv2.rectangle(view, (0, y0 + b0), (w - 1, y0 + b1), (0, 255, 0), 1)   # 초록: 띠
        for a, b in segs:                                                     # 노랑: 후보 구간
            color = (0, 0, 255) if best == (a, b) else (0, 255, 255)
            cv2.line(view, (a, y0 + b1 + 3), (max(a, b - 1), y0 + b1 + 3), color, 3)
        if x is not None:
            cv2.circle(view, (int(x), y0 + (b0 + b1) // 2), 5, (0, 0, 255), -1)  # 빨간 점
        st = self.status
        if st.get("state") in ("RUN", "HOLD"):
            cx, cy = w // 2, h - 15
            cv2.arrowedLine(view, (cx, cy), (int(cx + st.get("drive_steer", self.steer) * 60), cy - 30),
                            (0, 165, 255), 2)                                   # 주황: 실제 조향
            if st.get("noise"):
                cv2.arrowedLine(view, (cx, cy), (int(cx + self.steer * 60), cy - 30),
                                (255, 255, 255), 1)                             # 흰색: 정답 조향
        txt = f"{st.get('state', '')} x {'-' if x is None else int(x)} " \
              f"err {st.get('err') or 0:+.2f} steer {st.get('steer', 0):+.2f} {st.get('cmd', '')}"
        cv2.putText(view, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
        cv2.putText(view, f"line: {st.get('reason', '')}  cands {len(segs)}", (5, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
        d = st.get("dist_cm")
        zone_color = {"CAUTION": (0, 200, 255), "DANGER": (0, 0, 255), "EMERG": (0, 0, 255),
                      "NOSENSOR": (160, 160, 160)}.get(st.get("zone"))
        cv2.putText(view, f"dist {'-' if d is None else f'{d:.0f}cm'} {st.get('zone', '')}", (5, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, zone_color or (0, 165, 255), 1)
        if zone_color and st.get("safety"):
            cv2.rectangle(view, (1, 1), (w - 2, h - 2), zone_color, 3)          # 구간 색 테두리
        if self.recording:
            cv2.circle(view, (w - 12, 12), 6, (0, 0, 255), -1)
        # mask 화면: 흰색 = 선 후보로 남은 덩어리, 회색 = ① 에서 버린 덩어리
        m1 = np.where(mask > 0, 255, np.where(raw_mask > 0, 90, 0)).astype(np.uint8)
        m3 = cv2.cvtColor(m1, cv2.COLOR_GRAY2BGR)
        m3 = cv2.copyMakeBorder(m3, y0, h - y0 - mh, 0, 0, cv2.BORDER_CONSTANT, value=(40, 40, 40))
        ok, buf = cv2.imencode(".jpg", np.hstack([view, m3]), [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else None

    def loop(self):
        while not self.quit:
            try:
                frame = self.camera.read()
                with self.lock:
                    self.step(frame)
            except Exception as e:
                print(f"[오류] {e}")
                self.link.stop()
                time.sleep(0.2)


# =====================================================================
# 웹 화면
PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>cv_lab</title>
<style>body{font-family:sans-serif;margin:12px;background:#111;color:#eee}img{width:100%;max-width:960px}
button{font-size:18px;padding:8px 16px;margin:4px}.row{display:flex;align-items:center;gap:8px;max-width:640px}
.row label{width:70px}.row input{flex:1}#st{font-family:monospace;color:#fa0}
#view{position:relative;max-width:960px}#view img{display:block}
#banner{position:absolute;left:0;right:0;top:0;padding:8px 12px;font-size:20px;font-weight:bold;display:none}
#popup{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);padding:18px 28px;border-radius:12px;
 font-size:28px;font-weight:bold;text-align:center;display:none;border:4px solid #fff}
.caution{background:rgba(255,190,0,.85);color:#222}.danger{background:rgba(220,0,0,.9);color:#fff}
.nosensor{background:rgba(110,110,110,.9);color:#fff}
@keyframes blink{50%{opacity:.35}}.blink{animation:blink .4s step-end infinite}</style></head><body>
<div id="view"><img src="/stream"><div id="banner"></div><div id="popup"></div></div>
<div id="st">연결 중...</div>
<button id="run" onclick="act('run')">출발 (Space)</button>
<button id="rec" onclick="act('rec')">녹화 (R)</button>
<div id="sw" style="margin:6px 0"></div>
<div id="sl"></div>
<script>
const SL=__SLIDERS__, V=__SETTINGS__, SW=__SWITCHES__, IM=__IMPROVE__;
const swb=document.getElementById('sw');
SW.forEach(([k,lab])=>{swb.insertAdjacentHTML('beforeend',
 `<label style="margin-right:14px"><input type=checkbox ${IM[k]?'checked':''}
  onchange="fetch('/set?${k}='+(this.checked?1:0))"> ${lab}</label>`)});
const box=document.getElementById('sl');
SL.forEach(([n,a,b,s])=>{box.insertAdjacentHTML('beforeend',
 `<div class=row><label>${n}</label><input type=range min=${a} max=${b} step=${s} value=${V[n]}
  oninput="set('${n}',this.value)"><span id=v_${n}>${V[n]}</span></div>`)});
function set(n,v){document.getElementById('v_'+n).textContent=v;fetch('/set?'+n+'='+v)}
function act(a){fetch('/'+a,{method:'POST'})}
document.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();act('run')}
 if(e.code==='KeyR')act('rec')});
setInterval(async()=>{try{const s=await (await fetch('/state')).json();
 const why={ok:'찾음',none:'선 없음',width:'폭 안 맞음',jump:'점프 무시'}[s.reason]||s.reason;
 document.getElementById('st').textContent=`${s.state} | x ${s.x===null?'-':s.x.toFixed(0)} | 선 ${why} (후보 ${s.cands}) | steer ${s.steer.toFixed(2)} | ${s.cmd} | 모터 ${s.motor?'연결':'없음'} | 녹화 ${s.recording?'ON '+s.rec_count+'장':'OFF'}`;
 document.getElementById('run').textContent=s.running?'정지 (Space)':'출발 (Space)';
 document.getElementById('rec').textContent=s.recording?'녹화 중지 (R)':'녹화 (R)';
 warn(s);}catch(e){}},250);
function warn(s){const bn=document.getElementById('banner'),pp=document.getElementById('popup');
 const d=s.dist_cm===null?'-':Math.round(s.dist_cm)+' cm';
 document.getElementById('st').textContent+=` | 거리 ${d} ${s.zone}`+(s.speed_scale<1?` (속도 ${Math.round(s.speed_scale*100)}%)`:'')
  +(s.noise?` | 흔들기 ${s.noise>0?'+':''}${s.noise.toFixed(2)}`:'')+(s.recording&&s.rec_skipped?` | 건너뜀 ${s.rec_skipped}`:'');
 bn.style.display=pp.style.display='none';
 if(!s.safety)return;
 if(s.zone==='NOSENSOR'){pp.className='nosensor';pp.innerHTML='초음파 센서 확인<br><small>거리 정보 없음 · 정지</small>';pp.style.display='block';return}
 if(s.zone==='DANGER'||s.zone==='EMERG'||s.emergency){
  pp.className='danger blink';pp.innerHTML=`충돌 위험! 정지<br><small>전방 ${d}</small>`;pp.style.display='block';return}
 if(s.zone==='CAUTION'){bn.className='caution';bn.textContent=`전방 장애물 ${d} · 감속 중`;bn.style.display='block'}}
</script></body></html>"""


def create_app(driver):
    import json
    from flask import Flask, Response, request, jsonify
    app = Flask(__name__)

    @app.route("/")
    def index():
        return (PAGE.replace("__SLIDERS__", json.dumps(SLIDERS)).replace("__SETTINGS__", json.dumps(SETTINGS))
                .replace("__SWITCHES__", json.dumps(SWITCHES + SAFETY_SWITCHES + COLLECT_SWITCHES,
                                                    ensure_ascii=False))
                .replace("__IMPROVE__", json.dumps({**IMPROVE, **SAFETY, **COLLECT})))

    @app.route("/stream")
    def stream():
        def gen():
            while not driver.quit:
                jpg = driver.jpeg
                if jpg:
                    yield b"--f\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
                time.sleep(0.07)
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=f")

    @app.route("/state")
    def state():
        driver.ping()
        return jsonify(driver.status)

    @app.route("/run", methods=["POST"])
    def run():
        driver.ping()
        return jsonify(running=driver.toggle_run())

    @app.route("/rec", methods=["POST"])
    def rec():
        return jsonify(recording=driver.toggle_record())

    @app.route("/set")
    def set_value():
        for k, v in request.args.items():
            if k in SETTINGS:
                SETTINGS[k] = float(v)
            elif k in IMPROVE:
                IMPROVE[k] = 1 if v in ("1", "true") else 0
            elif k in SAFETY:
                SAFETY[k] = 1 if v in ("1", "true") else 0
            elif k in COLLECT:
                COLLECT[k] = 1 if v in ("1", "true") else 0
        return jsonify(settings=SETTINGS, improve=IMPROVE, safety=SAFETY, collect=COLLECT)

    return app


def main():
    link = MotorLink()
    camera = Camera()
    driver = Driver(camera, link)
    th = threading.Thread(target=driver.loop, daemon=True)
    th.start()
    print(f"[웹] 브라우저에서 http://<파이IP>:{WEB_PORT}  (IP 확인: hostname -I)   종료: Ctrl+C")
    try:
        import logging
        logging.getLogger("werkzeug").setLevel(logging.ERROR)
        create_app(driver).run(host="0.0.0.0", port=WEB_PORT, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        driver.quit = True
        driver.running = False
        time.sleep(0.2)
        link.close()
        camera.close()
        print("종료 (모터 정지)")


if __name__ == "__main__":
    main()
