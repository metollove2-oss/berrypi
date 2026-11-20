"""
OpenCV 선 따라가기 (임베디드시스템설계 11. cv_lab 기준, 베리파이 팀 완성본)

카메라 → [C1] make_mask → [C2] find_line_x → [C3] line_error → [C4] pd_steer
       → 커브 감속 → mix → to_command → 아두이노 (M,<좌>,<우>)
녹화 중이면 [C5] save_sample 로 사진 + labels.csv 저장 (학습 데이터)

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
]


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
                return
            except Exception as e:
                print(f"[모터] {p} 열기 실패: {e}")
        print("[모터] 아두이노를 찾을 수 없음 → 화면만 동작 (USB 연결, 시리얼 모니터 닫았는지 확인)")

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

    # ---- 웹에서 부르는 것
    def toggle_run(self):
        with self.lock:
            self.running = not self.running
            self.prev_t, self.lost = None, 0
            if not self.running:
                self.link.stop()
        return self.running

    def toggle_record(self):
        with self.lock:
            self.recording = not self.recording
            if self.recording:
                self.rec_folder = os.path.join("data", time.strftime("%Y%m%d_%H%M%S") + "_lab")
                self.rec_index = 0
                os.makedirs(self.rec_folder, exist_ok=True)
                with open(os.path.join(self.rec_folder, "info.json"), "w") as f:
                    import json
                    json.dump({"start": time.strftime("%Y-%m-%d %H:%M:%S"), "settings": dict(SETTINGS)},
                              f, ensure_ascii=False, indent=2)
                print(f"[녹화] 시작: {self.rec_folder}")
            else:
                print(f"[녹화] 정지: {self.rec_index}장")
        return self.recording

    def ping(self):
        self.last_ping = time.time()

    # ---- 한 프레임 처리
    def step(self, frame):
        s = dict(SETTINGS)
        mask, y0 = make_mask(frame, s["roi_top"], int(s["thresh"]))
        x = find_line_x(mask, s["look"])
        now = time.time()

        if self.running and now - self.last_ping > BROWSER_TIMEOUT:
            print("[안전] 브라우저 응답 없음 → 정지")
            self.running = False

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
            if state in ("RUN", "HOLD"):
                speed = s["speed"] * (1 - SLOWDOWN * abs(self.steer))
                cmd = to_command(*mix(speed, self.steer))
            if state == "RUN" and self.recording and now - self.rec_last >= 1.0 / RECORD_HZ:
                self.rec_index += 1
                save_sample(self.rec_folder, self.rec_index, frame, round(self.steer, 4), round(s["speed"], 3))
                self.rec_last = now
        self.link.send(cmd)                       # 매 프레임 전송 = 하트비트

        self.status = {"state": state, "x": x, "err": err, "steer": self.steer if state != "STOP" else 0.0,
                       "cmd": cmd.strip(), "lost": self.lost, "recording": self.recording,
                       "rec_count": self.rec_index, "running": self.running, "motor": self.link.ok}
        self.jpeg = self.draw(frame, mask, y0, x, s)

    def draw(self, frame, mask, y0, x, s):
        view = frame.copy()
        h, w = view.shape[:2]
        mh = mask.shape[0]
        cv2.line(view, (0, y0), (w, y0), (255, 255, 0), 1)                    # 하늘색: 자르는 위치
        b0 = int(np.clip(mh * s["look"] - mh * BAND / 2, 0, mh - 2))
        b1 = int(np.clip(b0 + mh * BAND, b0 + 2, mh))
        cv2.rectangle(view, (0, y0 + b0), (w - 1, y0 + b1), (0, 255, 0), 1)   # 초록: 띠
        if x is not None:
            cv2.circle(view, (int(x), y0 + (b0 + b1) // 2), 5, (0, 0, 255), -1)  # 빨간 점
        st = self.status
        if st.get("state") in ("RUN", "HOLD"):
            cx, cy = w // 2, h - 15
            cv2.arrowedLine(view, (cx, cy), (int(cx + self.steer * 60), cy - 30), (0, 165, 255), 2)
        txt = f"{st.get('state', '')} x {'-' if x is None else int(x)} " \
              f"err {st.get('err') or 0:+.2f} steer {st.get('steer', 0):+.2f} {st.get('cmd', '')}"
        cv2.putText(view, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
        if self.recording:
            cv2.circle(view, (w - 12, 12), 6, (0, 0, 255), -1)
        m3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
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
.row label{width:70px}.row input{flex:1}#st{font-family:monospace;color:#fa0}</style></head><body>
<img src="/stream"><div id="st">연결 중...</div>
<button id="run" onclick="act('run')">출발 (Space)</button>
<button id="rec" onclick="act('rec')">녹화 (R)</button>
<div id="sl"></div>
<script>
const SL=__SLIDERS__, V=__SETTINGS__;
const box=document.getElementById('sl');
SL.forEach(([n,a,b,s])=>{box.insertAdjacentHTML('beforeend',
 `<div class=row><label>${n}</label><input type=range min=${a} max=${b} step=${s} value=${V[n]}
  oninput="set('${n}',this.value)"><span id=v_${n}>${V[n]}</span></div>`)});
function set(n,v){document.getElementById('v_'+n).textContent=v;fetch('/set?'+n+'='+v)}
function act(a){fetch('/'+a,{method:'POST'})}
document.addEventListener('keydown',e=>{if(e.code==='Space'){e.preventDefault();act('run')}
 if(e.code==='KeyR')act('rec')});
setInterval(async()=>{try{const s=await (await fetch('/state')).json();
 document.getElementById('st').textContent=`${s.state} | x ${s.x===null?'-':s.x.toFixed(0)} | steer ${s.steer.toFixed(2)} | ${s.cmd} | 모터 ${s.motor?'연결':'없음'} | 녹화 ${s.recording?'ON '+s.rec_count+'장':'OFF'}`;
 document.getElementById('run').textContent=s.running?'정지 (Space)':'출발 (Space)';
 document.getElementById('rec').textContent=s.recording?'녹화 중지 (R)':'녹화 (R)';}catch(e){}},300);
</script></body></html>"""


def create_app(driver):
    import json
    from flask import Flask, Response, request, jsonify
    app = Flask(__name__)

    @app.route("/")
    def index():
        return PAGE.replace("__SLIDERS__", json.dumps(SLIDERS)).replace("__SETTINGS__", json.dumps(SETTINGS))

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
        return jsonify(SETTINGS)

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
