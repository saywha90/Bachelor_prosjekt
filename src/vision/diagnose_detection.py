"""
Live HSV Diagnose for OAK kamera
===================================
Viser røde og blå HSV-masker live, slik at vi kan se
nøyaktig hva kameraet faktisk oppfatter.

Klikk i Original-vinduet for å lese av HSV-verdier på det piksel.

Kontroller:
  Klikk  — les av HSV-verdi under musepekeren
  'q'    — avslutt
  'c'    — skriv ut terskelverdier til konsoll

Bruk: python src/vision/diagnose_detection.py
"""

import cv2
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision.oak_camera import OAKCamera
from vision.enhanced_detector import SimpleBallDetector
import config

# ─── Globals for mouse callback ──────────────────────────────────────────────
_last_hsv = None
_hsv_frame_global = None


def _on_mouse(event, x, y, flags, param):
    global _last_hsv
    if event == cv2.EVENT_LBUTTONDOWN and _hsv_frame_global is not None:
        h_fr, w_fr = _hsv_frame_global.shape[:2]
        x = max(0, min(x, w_fr - 1))
        y = max(0, min(y, h_fr - 1))
        h, s, v = _hsv_frame_global[y, x]
        _last_hsv = (int(h), int(s), int(v))
        print(f"  [KLIKK]  x={x}, y={y}  →  H={h}  S={s}  V={v}")


def _build_combined_mask(hsv: np.ndarray, ranges) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    kern_s = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kern_l = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern_s)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern_l)
    return mask


def _count_valid_contours(mask: np.ndarray, min_r: int, max_r: int) -> int:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    n = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area < np.pi * (min_r ** 2):
            continue
        _, r = cv2.minEnclosingCircle(c)
        if min_r <= r <= max_r:
            n += 1
    return n


def _contour_rejection_stats(mask: np.ndarray, min_r: int, max_r: int, label: str = ""):
    """Print why contours are failing validation."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total = len(contours)
    fail_area = fail_radius = fail_circ = fail_ar = fail_sol = 0
    passed = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < np.pi * (min_r ** 2):
            fail_area += 1
            continue
        (_, _), r = cv2.minEnclosingCircle(c)
        if not (min_r <= r <= max_r):
            fail_radius += 1
            continue
        perim = cv2.arcLength(c, True)
        circ = (4 * np.pi * area) / (perim ** 2) if perim > 0 else 0
        if circ < 0.45:
            fail_circ += 1
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        ar = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0
        if ar < 0.60:
            fail_ar += 1
            continue
        hull = cv2.convexHull(c)
        ha = cv2.contourArea(hull)
        sol = area / ha if ha > 0 else 0
        if sol < 0.60:
            fail_sol += 1
            continue
        passed += 1

    print(f"  [{label}] {total} contours råa  →  "
          f"fail area={fail_area}  radius={fail_radius}  "
          f"circ={fail_circ}  ar={fail_ar}  sol={fail_sol}  "
          f"GODKJENT={passed}")


def main():
    global _hsv_frame_global

    detector = SimpleBallDetector(min_radius=10, max_radius=150)

    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)
    cam.open()
    if not cam.isOpened():
        print("FEIL: Klarte ikke åpne OAK kamera")
        return

    print("OAK kamera åpnet. Klikk i Original-vinduet for å lese HSV-verdier.")
    print("Trykk 'q' for å avslutte, 'c' for å printe terskelverdier.\n")

    WIN_ORIG = "Original (klikk=HSV)"
    WIN_RED  = "Rød maske"
    WIN_BLUE = "Blå maske"
    WIN_OVER = "Overlay (rød=rød, blå=blå)"

    for w in [WIN_ORIG, WIN_RED, WIN_BLUE, WIN_OVER]:
        cv2.namedWindow(w, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(w, 640, 360)

    cv2.setMouseCallback(WIN_ORIG, _on_mouse)

    # Plasser vinduer side-om-side
    cv2.moveWindow(WIN_ORIG,  0,   0)
    cv2.moveWindow(WIN_RED,  650,  0)
    cv2.moveWindow(WIN_BLUE,   0, 400)
    cv2.moveWindow(WIN_OVER, 650, 400)

    frame_n = 0

    try:
        while True:
            ok, frame = cam.read()
            if not ok or frame is None:
                continue

            frame_n += 1

            # ─── HSV ──────────────────────────────────────────────────────────
            hsv_raw = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_blur = cv2.GaussianBlur(hsv_raw, (5, 5), 0)

            _hsv_frame_global = hsv_raw  # for mouse callback (no blur — true values)

            # ─── Masker ───────────────────────────────────────────────────────
            red_mask  = _build_combined_mask(hsv_blur, detector.red_ranges)
            blue_mask = _build_combined_mask(hsv_blur, detector.blue_ranges)

            n_red  = _count_valid_contours(red_mask,  detector.min_radius, detector.max_radius)
            n_blue = _count_valid_contours(blue_mask, detector.min_radius, detector.max_radius)

            # ─── Visualisering ────────────────────────────────────────────────
            # Fargede masker
            red_vis  = cv2.merge([np.zeros_like(red_mask),  np.zeros_like(red_mask),  red_mask])
            blue_vis = cv2.merge([blue_mask, np.zeros_like(blue_mask), np.zeros_like(blue_mask)])

            # Overlay på original
            overlay = frame.copy()
            overlay[red_mask > 0]  = overlay[red_mask > 0]  * 0.4 + np.array([0,   0, 220]) * 0.6
            overlay[blue_mask > 0] = overlay[blue_mask > 0] * 0.4 + np.array([220, 0,   0]) * 0.6

            # Statistikk-tekst
            def put(img, text, y, color=(255, 255, 255)):
                cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 3)
                cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color,  2)

            put(frame,   f"Frame #{frame_n}  |  klikk for HSV-verdi", 30)
            if _last_hsv:
                h, s, v = _last_hsv
                put(frame, f"Siste klikk: H={h} S={s} V={v}", 62, (0, 255, 255))

            put(red_vis,  f"ROD MASKE  — {n_red} gyldige contours", 30, (0, 100, 255))
            put(blue_vis, f"BLA MASKE  — {n_blue} gyldige contours", 30, (255, 200, 0))
            put(overlay,  f"Rod={n_red}  Bla={n_blue}", 30)

            cv2.imshow(WIN_ORIG, frame)
            cv2.imshow(WIN_RED,  red_vis)
            cv2.imshow(WIN_BLUE, blue_vis)
            cv2.imshow(WIN_OVER, overlay)

            # ─── Konsoll-output hvert 30. frame ──────────────────────────────
            if frame_n % 30 == 0:
                # Beregn piksel-andel fanget av maskene
                total_px = frame.shape[0] * frame.shape[1]
                red_pct  = red_mask.sum() // 255 / total_px * 100
                blue_pct = blue_mask.sum() // 255 / total_px * 100
                print(f"  Frame {frame_n:4d}:  RØD maske {red_pct:.1f}% av piksler  |  "
                      f"BLÅ maske {blue_pct:.1f}% av piksler  |  "
                      f"contours rød={n_red} blå={n_blue}")
                # Detaljer om HVORFOR contours feiler
                _contour_rejection_stats(red_mask,  detector.min_radius, detector.max_radius, "RØD")
                _contour_rejection_stats(blue_mask, detector.min_radius, detector.max_radius, "BLÅ")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('c'):
                print("\n=== NÅVÆRENDE TERSKLER ===")
                print("Rød ranges:")
                for lo, hi in detector.red_ranges:
                    print(f"  H=[{lo[0]}-{hi[0]}] S=[{lo[1]}-{hi[1]}] V=[{lo[2]}-{hi[2]}]")
                print("Blå ranges:")
                for lo, hi in detector.blue_ranges:
                    print(f"  H=[{lo[0]}-{hi[0]}] S=[{lo[1]}-{hi[1]}] V=[{lo[2]}-{hi[2]}]")
                print()

    except KeyboardInterrupt:
        pass
    finally:
        cam.release()
        cv2.destroyAllWindows()
        print("\nAvsluttet.")


if __name__ == "__main__":
    main()
