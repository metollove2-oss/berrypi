"""자동 데이터 수집 기능 자동 확인 (카메라 · 모터 없이, PC에서도 실행 가능)
- 자동 흔들기: 실제 조향에는 잡음이 섞이고, 저장되는 정답은 잡음 없는 조향인지
- 확실할 때만 저장: 후보가 2개이거나 점프 직후인 프레임을 건너뛰는지
실행: python3 check_collect.py
"""
import csv
import importlib.util
import os
import tempfile
import time

import cv2
import numpy as np

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cv_lab.py")
spec = importlib.util.spec_from_file_location("cv_lab", path)
lab = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)
os.chdir(tempfile.mkdtemp())

ok = total = 0


def check(name, cond, info=""):
    global ok, total
    total += 1
    ok += bool(cond)
    print(f"  {'[통과]' if cond else '[실패]'} {name}  {info}")


class FakeLink:
    ok = True

    def __init__(self):
        self.sent, self.dist_mm, self.dist_time, self.emergency = [], 2000, time.time() + 1e9, False

    def send(self, t):
        self.sent.append(t.strip())

    def stop(self):
        self.send("M,0,0\n")


def scene(xs=(160,)):
    im = np.full((240, 320, 3), 200, np.uint8)
    for x in xs:
        cv2.rectangle(im, (x - 6, 0), (x + 6, 239), (20, 20, 20), -1)
    return im


def wheels(link):
    m = [t for t in link.sent if t.startswith("M,")][-1]
    return [int(v) for v in m.split(",")[1:]]


def new_driver():
    link = FakeLink()
    d = lab.Driver(camera=None, link=link)
    d.ping()
    d.toggle_run()
    return d, link


def step(d, im):
    d.ping()
    d.step(im)
    return d.status


print("자동 흔들기")
lab.COLLECT.update(noise=0, confident=1)
d, link = new_driver()
st = step(d, scene())
l, r = wheels(link)
check("흔들기 끔 → 가운데 선이면 좌우 같은 속도", l == r and st["noise"] == 0, f"M,{l},{r}")

lab.COLLECT["noise"] = 1
d.noise, d.noise_until = 0.3, time.time() + 100          # 잡음 +0.3 고정
d.toggle_record()
d.rec_last = 0
st = step(d, scene())
l, r = wheels(link)
d.toggle_record()
rows = list(csv.reader(open(os.path.join(d.rec_folder, "labels.csv"))))
label = float(rows[1][1])
check("흔들기 켬 → 실제 명령은 오른쪽으로 꺾음 (왼쪽 바퀴가 빠름)", l > r, f"M,{l},{r} 실제 조향 {st['drive_steer']:+.2f}")
check("저장된 정답은 잡음 없는 조향 (≈ 0)", abs(label) < 0.05 and abs(st["drive_steer"] - label - 0.3) < 0.05,
      f"정답 {label:+.3f}")

link.dist_mm, link.dist_time = 250, time.time()           # 주의 구간 → 감속 중
st = step(d, scene())
check("감속 중(주의 구간)에는 흔들지 않음", st["noise"] == 0 and st["speed_scale"] < 1,
      f"잡음 {st['noise']} 속도 {st['speed_scale']}")
link.dist_mm = 2000

vals = []
d.noise_until = 0
t = 0.0
for _ in range(2000):
    t += 1.0
    vals.append(d.next_noise(t, 0.3))
vals = np.array(vals)
zero = np.mean(vals == 0)
check("잡음: 절반 정도는 0, 크기는 ±0.3 안", 0.4 < zero < 0.6 and np.abs(vals).max() <= 0.3,
      f"0 비율 {zero:.2f}, 최대 {np.abs(vals).max():.2f}")
lab.COLLECT["noise"] = 0

print("확실할 때만 저장")


def record(d, frames, gap=True):
    d.toggle_record()
    for im in frames:
        d.rec_last = 0 if gap else d.rec_last
        step(d, im)
    n, skip = d.rec_index, d.rec_skipped
    d.toggle_record()
    return n, skip


d, link = new_driver()
step(d, scene())
n, skip = record(d, [scene()] * 3)
check("선 하나 → 3장 모두 저장", n == 3 and skip == 0, f"저장 {n}, 건너뜀 {skip}")
n, skip = record(d, [scene((160, 80))] * 3)
check("옆에 다른 테이프(후보 2개) → 건너뜀", n == 0 and skip == 3, f"저장 {n}, 건너뜀 {skip}")
lab.COLLECT["confident"] = 0
n, skip = record(d, [scene((160, 80))] * 3)
check("확실할 때만 저장 끔 → 후보 2개여도 저장", n == 3, f"저장 {n}")
lab.COLLECT["confident"] = 1

d, link = new_driver()
for _ in range(3):
    step(d, scene((160,)))
n, skip = record(d, [scene((60,))] * 3)
check("점프 직후 프레임 → 건너뜀", n == 0 and skip >= 1, f"저장 {n}, 건너뜀 {skip}")
print(f"결과: 통과 {ok} / {total}")
