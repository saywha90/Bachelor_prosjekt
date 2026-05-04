"""
HSV rekalibrering fra treningsbilder.
Filtrerer ut bakgrunnspikslene (svart/hvit bakgrunn) for å få riktige ball-verdier.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import cv2
import numpy as np
from pathlib import Path


def analyze_color(data_dir: str, color: str):
    path = Path(data_dir) / color
    files = list(path.glob("*.jpg")) + list(path.glob("*.png"))
    print(f"\n=== {color.upper()} ({len(files)} bilder) ===")

    all_h, all_s, all_v = [], [], []
    skipped = 0
    for f in sorted(files):
        img = cv2.imread(str(f))
        if img is None:
            print(f"  SKIP (lesefeil): {f.name}")
            skipped += 1
            continue
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        # Filtrer ut bakgrunn: S>50 og V>40 ekskluderer svart og hvit bakgrunn
        mask = (s > 50) & (v > 40)
        if mask.sum() < 50:
            skipped += 1
            continue
        all_h.extend(h[mask].tolist())
        all_s.extend(s[mask].tolist())
        all_v.extend(v[mask].tolist())

    if not all_h:
        print("  Ingen data!")
        return None

    if skipped:
        print(f"  ({skipped} bilder hoppet over)")

    all_h = np.array(all_h)
    all_s = np.array(all_s)
    all_v = np.array(all_v)

    print(f"  Totalt ball-piksler: {len(all_h):,}")
    print(f"  H: p2={np.percentile(all_h, 2):.0f}  p5={np.percentile(all_h, 5):.0f}  p50={np.percentile(all_h, 50):.0f}  p95={np.percentile(all_h, 95):.0f}  p98={np.percentile(all_h, 98):.0f}")
    print(f"  S: p5={np.percentile(all_s, 5):.0f}  p10={np.percentile(all_s, 10):.0f}  p50={np.percentile(all_s, 50):.0f}  p95={np.percentile(all_s, 95):.0f}")
    print(f"  V: p5={np.percentile(all_v, 5):.0f}  p10={np.percentile(all_v, 10):.0f}  p50={np.percentile(all_v, 50):.0f}  p95={np.percentile(all_v, 95):.0f}")

    low_h = (all_h <= 15).sum()
    high_h = (all_h >= 160).sum()
    mid_h = len(all_h) - low_h - high_h
    print(f"  H-fordeling: lav(0-15)={low_h/len(all_h)*100:.0f}%  mid(16-159)={mid_h/len(all_h)*100:.0f}%  høy(160-179)={high_h/len(all_h)*100:.0f}%")

    return {"h": all_h, "s": all_s, "v": all_v}


def suggest_red_ranges(data):
    h, s, v = data["h"], data["s"], data["v"]

    s_p5  = int(np.percentile(s, 5))
    s_p10 = int(np.percentile(s, 10))
    s_p20 = int(np.percentile(s, 20))

    v_p5  = int(np.percentile(v, 5))
    v_p10 = int(np.percentile(v, 10))
    v_p20 = int(np.percentile(v, 20))

    print(f"""
=== FORSLAG TIL RØD HSV-RANGES ===
(3 nivåer: lys, medium, mørk)

    # Lys (godt lys) - S>={int(s_p5*1.4)}, V>={int(v_p5*1.4)}
    (np.array([0,   {min(255, int(s_p5*1.4))}, {min(255, int(v_p5*1.4))}]), np.array([11,  255, 255])),
    (np.array([168, {min(255, int(s_p5*1.4))}, {min(255, int(v_p5*1.4))}]), np.array([179, 255, 255])),
    # Medium - S>={s_p10}, V>={v_p10}
    (np.array([0,   {s_p10}, {v_p10}]), np.array([11,  255, 255])),
    (np.array([168, {s_p10}, {v_p10}]), np.array([179, 255, 255])),
    # Mørk - S>={s_p20}, V>={v_p20}
    (np.array([0,   {s_p20}, {v_p20}]), np.array([11,  255, 175])),
    (np.array([168, {s_p20}, {v_p20}]), np.array([179, 255, 175])),
""")


def suggest_blue_ranges(data):
    h, s, v = data["h"], data["s"], data["v"]

    h_p5  = int(np.percentile(h, 5))
    h_p95 = int(np.percentile(h, 95))
    s_p5  = int(np.percentile(s, 5))
    s_p20 = int(np.percentile(s, 20))
    v_p5  = int(np.percentile(v, 5))
    v_p20 = int(np.percentile(v, 20))

    print(f"""
=== FORSLAG TIL BLÅ HSV-RANGES ===
    H-senter: p5={h_p5}, p95={h_p95}
    # Lys - S>={int(s_p5*1.3)}, V>={int(v_p5*1.3)}
    (np.array([{max(90, h_p5-5)},  {min(255, int(s_p5*1.3))}, {min(255, int(v_p5*1.3))}]), np.array([{min(130, h_p95+5)}, 255, 255])),
    # Medium - S>={s_p5}, V>={v_p5}
    (np.array([{max(85, h_p5-10)}, {s_p5}, {v_p5}]), np.array([{min(135, h_p95+10)}, 255, 255])),
    # Mørk - S>={s_p20}, V>={v_p20}
    (np.array([{max(80, h_p5-15)}, {s_p20}, {v_p20}]), np.array([{min(140, h_p95+15)}, 255, 220])),
""")


if __name__ == "__main__":
    data_dir = sys.argv[1] if len(sys.argv) > 1 else "training_data"

    red_data  = analyze_color(data_dir, "red")
    blue_data = analyze_color(data_dir, "blue")

    if red_data:
        suggest_red_ranges(red_data)
    if blue_data:
        suggest_blue_ranges(blue_data)
