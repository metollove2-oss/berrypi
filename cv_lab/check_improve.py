"""선 검출 개선 ① ~ ④ 자동 확인 (카메라 · 모터 없이, PC에서도 실행 가능)
합성 사진으로 '개선 끔'과 '개선 켬' 결과를 나란히 보여 준다.
실행: python3 check_improve.py
"""
import importlib.util
import os
import sys
import time
import types

import cv2
import numpy as np

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cv_lab.py")
spec = importlib.util.spec_from_file_location("cv_lab", path)
lab = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)

ok = total = 0


def check(name, cond, info):
    global ok, total
    total += 1
    ok += bool(cond)
    print(f"  {'[통과]' if cond else '[실패]'} {name}  {info}")


def scene(line_x=200, extra=()):
    im = np.full((240, 320, 3), 200, np.uint8)
    if line_x is not None:
        cv2.rectangle(im, (line_x - 6, 0), (line_x + 6, 239), (20, 20, 20), -1)
    for (x0, y0, x1, y1) in extra:
        cv2.rectangle(im, (x0, y0), (x1, y1), (20, 20, 20), -1)
    return im


def run(im, imp, prev_x=None, s=None):
    s = s or dict(lab.SETTINGS)
    raw, _ = lab.make_mask(im, s["roi_top"], int(s["thresh"]))
    mask = lab.keep_bottom(raw, s["min_bottom"]) if imp["bottom"] else raw
    x, segs, _ = lab.detect_line(mask, s["look"], prev_x, imp["width"], imp["nearest"], s["max_w"])
    return x


OFF = {"bottom": 0, "width": 0, "nearest": 0, "jump": 0}
ON = {"bottom": 1, "width": 1, "nearest": 1, "jump": 1}
fmt = lambda v: "None" if v is None else f"{v:.0f}"

print("① 아래 연결: 띠에 걸친 '떠 있는' 검은 물체 (x 40~70, 아래쪽에 안 닿음)")
im = scene(200, [(40, 170, 70, 200)])
a, b = run(im, OFF), run(im, dict(OFF, bottom=1))
check("물체 무시하고 선 x 200", b is not None and abs(b - 200) < 2, f"끔 {fmt(a)} → 켬 {fmt(b)}")

print("② 선 폭: 화면 왼쪽의 넓은 그림자 (x 0~120, 바닥까지 이어짐)")
im = scene(200, [(0, 108, 120, 239)])
a, b = run(im, OFF), run(im, dict(OFF, bottom=1, width=1))
check("그림자 무시하고 선 x 200", b is not None and abs(b - 200) < 2, f"끔 {fmt(a)} → 켬 {fmt(b)}")

print("③ 가까운 선: 옆에 다른 테이프 (x 80), 지난 위치 x 195")
im = scene(200, [(74, 108, 86, 239)])
a, b = run(im, OFF, 195), run(im, ON, 195)
check("지난 위치에 가까운 x 200 선택", b is not None and abs(b - 200) < 2, f"끔 {fmt(a)} → 켬 {fmt(b)}")

print("기본 동작이 그대로인지 (방해물 없음)")
for xx in (60, 160, 260):
    b = run(scene(xx), ON)
    check(f"선 x {xx}", b is not None and abs(b - xx) < 2, f"켬 {fmt(b)}")
check("선 없음 → None", run(scene(None), ON) is None, f"켬 {fmt(run(scene(None), ON))}")
im = scene(None, [(150, 108, 162, 239)])
cv2.line(im, (160, 239), (300, 108), (20, 20, 20), 12)   # 오른쪽으로 휘는 커브
b = run(im, ON, 200)
check("기운 선(커브)도 찾음", b is not None, f"켬 {fmt(b)}")

print("④ 점프 무시: 선이 x 200에서 갑자기 x 60으로 (주행 반복에서 확인)")


class FakeLink:
    ok = True
    dist_mm, dist_time, emergency = 2000, time.time() + 1e9, False   # 충돌 방지와 무관하게 '먼 거리'
    def send(self, t): pass
    def stop(self): pass


d = lab.Driver(camera=None, link=FakeLink())
d.ping()
xs = []
for im in [scene(200)] * 3 + [scene(60)] * 4:
    d.step(im)
    xs.append(d.status["x"])
seq = [fmt(v) for v in xs]
check("튄 첫 2프레임은 무시, 3번째부터 인정", xs[3] is None and xs[4] is None and xs[5] is not None
      and abs(xs[5] - 60) < 2, " → ".join(seq))

print("속도")
im = scene(200, [(40, 170, 70, 200), (0, 108, 120, 239)])
t = time.time()
for _ in range(100):
    d.step(im)
ms = (time.time() - t) * 10
check("한 프레임 처리(화면 그리기 포함) 15 ms 이하", ms < 15, f"{ms:.1f} ms")
print(f"결과: 통과 {ok} / {total}")
