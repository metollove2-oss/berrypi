"""충돌 방지 기능 자동 확인 (카메라 · 모터 · 센서 없이, PC에서도 실행 가능)
가짜 아두이노가 보내는 거리로 구간 판단 · 감속 · 정지 · 재출발 · 아두이노 설정 전송을 확인한다.
실행: python3 check_safety.py
"""
import importlib.util
import os
import time

import cv2
import numpy as np

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cv_lab.py")
spec = importlib.util.spec_from_file_location("cv_lab", path)
lab = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)

ok = total = 0


def check(name, cond, info=""):
    global ok, total
    total += 1
    ok += bool(cond)
    print(f"  {'[통과]' if cond else '[실패]'} {name}  {info}")


print("구간 판단 safety_zone (함수 시험: 주의 50 / 위험 25 / 비상 12 cm)")
Z = lambda mm, prev="SAFE": lab.safety_zone(mm, prev, 50, 25, 12)
for mm, prev, exp in [(2000, "SAFE", "SAFE"), (450, "SAFE", "CAUTION"), (200, "SAFE", "DANGER"),
                      (100, "SAFE", "EMERG"), (270, "DANGER", "DANGER"), (310, "DANGER", "CAUTION"),
                      (520, "CAUTION", "CAUTION"), (560, "CAUTION", "SAFE"), (-1, "SAFE", "NOSENSOR"),
                      (None, "SAFE", "NOSENSOR")]:
    check(f"{mm} mm (이전 {prev}) → {exp}", Z(mm, prev) == exp, Z(mm, prev))

print("감속 비율 safety_speed_scale")
S = lambda mm: lab.safety_speed_scale(mm, 50, 25)
check("50 cm → 1.0, 25 cm → 0.3, 37.5 cm → 0.65",
      abs(S(500) - 1) < 1e-9 and abs(S(250) - 0.3) < 1e-9 and abs(S(375) - 0.65) < 1e-9,
      f"{S(500):.2f} {S(250):.2f} {S(375):.2f}")


class FakeLink:
    ok = True

    def __init__(self):
        self.sent, self.dist_mm, self.dist_time, self.emergency = [], 2000, time.time(), False

    def send(self, t):
        self.sent.append(t.strip())

    def stop(self):
        self.send("M,0,0\n")

    def feed(self, mm):
        self.dist_mm, self.dist_time = mm, time.time()


def scene():
    im = np.full((240, 320, 3), 200, np.uint8)
    cv2.rectangle(im, (154, 0), (166, 239), (20, 20, 20), -1)    # 가운데 선
    return im


def last_m(link):
    return [t for t in link.sent if t.startswith("M,")][-1]


def wheels(cmd):
    return [int(v) for v in cmd.split(",")[1:]]


print("주행 반복 (가짜 아두이노)")
link = FakeLink()
d = lab.Driver(camera=None, link=link)
d.ping()
d.toggle_run()
frame = scene()


def step(mm=None):
    if mm is not None:
        link.feed(mm)
    d.ping()
    d.step(frame)
    return d.status


st = step(2000)
safe_l = wheels(last_m(link))[0]
check("멀면 정상 주행", st["state"] == "RUN" and safe_l > 0, f"{st['state']} {last_m(link)}")
check("ARM,1 · MUTE,0 · TH,300,150,80 전송", {"ARM,1", "MUTE,0", "TH,300,150,80"} <= set(link.sent))
st = step(250)
l = wheels(last_m(link))[0]
check("25 cm → 주의, 감속", st["zone"] == "CAUTION" and 0 < l < safe_l and st["speed_scale"] < 1,
      f"{st['zone']} {last_m(link)} 속도 {st['speed_scale']}")
st = step(120)
check("12 cm → 위험, OBST 정지", st["zone"] == "DANGER" and st["state"] == "OBST" and last_m(link) == "M,0,0",
      f"{st['zone']} {st['state']} {last_m(link)}")
st = step(170)
check("17 cm (여유 구간 15~18) → 아직 정지", st["state"] == "OBST", st["state"])
st = step(250)
check("25 cm로 치워도 0.5초 전에는 정지 유지", st["state"] == "OBST", st["state"])
time.sleep(lab.RESUME_HOLD + 0.05)
st = step(250)
check("0.5초 지나면 자동 재출발 (주의 구간이라 감속)", st["state"] == "RUN" and wheels(last_m(link))[0] > 0,
      f"{st['state']} {last_m(link)}")

link.emergency = True
st = step(250)
check("아두이노 비상 정지(E,1) → OBST", st["state"] == "OBST", st["state"])
link.emergency = False
time.sleep(lab.RESUME_HOLD + 0.05)
step(2000)

lab.SAFETY["auto_resume"] = 0
step(120)
step(2000)
time.sleep(lab.RESUME_HOLD + 0.05)
st = step(2000)
check("자동 재출발 끔 → 치워도 정지, Space 기다림", not st["running"] and st["state"] == "STOP",
      f"running={st['running']} {st['state']}")
lab.SAFETY["auto_resume"] = 1
d.toggle_run()

link.dist_time = time.time() - 2
st = step()
check("거리 메시지 1초 끊김 → 센서 오류로 정지", st["zone"] == "NOSENSOR" and st["state"] == "OBST",
      f"{st['zone']} {st['state']}")

lab.SAFETY["safety"] = 0
st = step()
check("충돌 방지 끔 → 센서 없어도 주행", st["state"] == "RUN" and "ARM,0" in link.sent, st["state"])
lab.SAFETY["safety"] = 1
lab.SAFETY["sound"] = 0
step(2000)
check("경고음 끔 → MUTE,1 전송", "MUTE,1" in link.sent)
lab.SAFETY["sound"] = 1

print("녹화: 장애물 정지 중에는 저장 안 함")
import tempfile
os.chdir(tempfile.mkdtemp())
d.toggle_record()
d.rec_last = 0
step(120)
step(120)
n = d.rec_index
d.toggle_record()
check("OBST 동안 0장", n == 0, f"{n}장")
print(f"결과: 통과 {ok} / {total}")
