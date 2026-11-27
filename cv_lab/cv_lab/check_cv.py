"""C1 ~ C5 자동 확인 (카메라 · 모터 없이, PC에서도 실행 가능)
실행: python3 check_cv.py            # 같은 폴더의 cv_lab.py
      python3 check_cv.py 다른파일.py
"""
import csv
import importlib.util
import os
import sys
import tempfile

import cv2
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "cv_lab.py")
spec = importlib.util.spec_from_file_location("cv_lab", path)
lab = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lab)

ok = total = 0


def check(name, fn):
    global ok, total
    total += 1
    try:
        res, info = fn()
        tag = "[통과]" if res else "[실패]"
        ok += bool(res)
    except NotImplementedError:
        tag, info = "[아직]", "(함수를 아직 작성하지 않음)"
    except Exception as e:
        tag, info = "[오류]", f"{type(e).__name__}: {e}"
    print(f"  {tag} {name} {'' if tag == '[통과]' else info}")


def img(x=None, pts=None):
    im = np.full((240, 320, 3), 200, np.uint8)
    if x is not None:
        cv2.rectangle(im, (x - 6, 0), (x + 6, 239), (20, 20, 20), -1)
    if pts:
        cv2.line(im, pts[0], pts[1], (20, 20, 20), 12)
    return im


print("C1 make_mask(bgr, roi_top, thresh)")
check("반환 (mask, y0) · 크기", lambda: ((lambda m, y: (m.shape == (240 - y, 320) and y == 108, (m.shape, y)))
                                    (*lab.make_mask(img(200), 0.45, 80))))
check("선 = 255, 바닥 = 0", lambda: ((lambda m: (m[50, 200] == 255 and m[50, 50] == 0,
                                                f"선 {m[50, 200]}, 바닥 {m[50, 50]} (THRESH_BINARY_INV?)"))
                                    (lab.make_mask(img(200), 0.45, 80)[0])))
gray100 = np.full((240, 320, 3), 100, np.uint8)
check("thresh 적용", lambda: (lab.make_mask(gray100, 0.45, 80)[0].max() == 0
                             and lab.make_mask(gray100, 0.45, 120)[0].min() == 255, "thresh 인자를 썼는지"))
dot = img()
dot[150, 100] = 0
check("1픽셀 잡티 제거", lambda: (lab.make_mask(dot, 0.45, 80)[0].max() == 0, "MORPH_OPEN 3x3"))

print("C2 find_line_x(mask, look)")
for xx in (200, 60):
    check(f"선 x {xx}", lambda xx=xx: ((lambda r: (r is not None and abs(r - xx) < 1, r))
                                       (lab.find_line_x(lab.make_mask(img(xx), 0.45, 80)[0], 0.6))))
check("선 없음 None", lambda: ((lambda r: (r is None, r))(lab.find_line_x(lab.make_mask(img(), 0.45, 80)[0], 0.6))))
tilt = lab.make_mask(img(pts=((60, 108), (260, 239))), 0.45, 80)[0]
check("기운 선 look 0.2", lambda: ((lambda r: (r is not None and 85 < r < 115, r))(lab.find_line_x(tilt, 0.2))))
check("기운 선 look 0.9", lambda: ((lambda r: (r is not None and 225 < r < 255, r))(lab.find_line_x(tilt, 0.9))))
outside = np.zeros((132, 320), np.uint8)
outside[0:20, 100:110] = 255
check("띠 밖의 선 무시", lambda: ((lambda r: (r is None, r))(lab.find_line_x(outside, 0.6))))

print("C3 line_error(x, width)")
check("중심 0", lambda: (lab.line_error(160, 320) == 0, lab.line_error(160, 320)))
check("왼쪽 끝 -1", lambda: (lab.line_error(0, 320) == -1, lab.line_error(0, 320)))
check("오른쪽 끝 +1", lambda: (lab.line_error(320, 320) == 1, lab.line_error(320, 320)))
check("x 240 → 0.5", lambda: (lab.line_error(240, 320) == 0.5, lab.line_error(240, 320)))

print("C4 pd_steer(error, prev_error, dt, kp, kd)")
for name, a, exp in [("변화 없음", (0.5, 0.5, 0.05, 1, 0.5), 0.5), ("벗어나는 중", (0.5, 0.4, 0.1, 1, 0.5), 0.55),
                     ("0.1초 기준", (0.5, 0.4, 0.05, 1, 0.5), 0.6), ("돌아오는 중", (0.3, 0.5, 0.1, 1, 0.5), 0.2),
                     ("자르기", (0.8, 0.0, 0.05, 2, 0.5), 1.0), ("dt 0", (0.5, 0.1, 0, 1, 0.5), 0.5)]:
    check(name, lambda a=a, exp=exp: ((lambda r: (abs(r - exp) < 1e-6, f"{r}, 기대 {exp}"))(lab.pd_steer(*a))))

print("C5 save_sample(folder, index, frame, steering, throttle)")


def c5():
    d = tempfile.mkdtemp()
    n1 = lab.save_sample(d, 1, img(160), 0.25, 0.35)
    n2 = lab.save_sample(d, 2, img(160), -0.5, 0.35)
    rows = list(csv.reader(open(os.path.join(d, "labels.csv"))))
    good = (n1 == "images/000001.jpg" and os.path.exists(os.path.join(d, n2)) and len(rows) == 3
            and rows[0] == ["image", "steering", "throttle", "time"])
    return good, rows


check("사진 2장 + labels.csv 3줄", c5)
print(f"결과: 통과 {ok} / {total}")
