#!/usr/bin/env python3
"""
stream_debug.py — Live 4-panel debug-vindu for OAK-kamera
==========================================================

Kjøres på Pi (via VNC):  python src/vision/stream_debug.py

4-panel debug-bilde vist via cv2.imshow:
  ┌────────────────────┬─────────────────────┐
  │  Kamera +          │  Rød HSV-maske      │
  │  deteksjoner (●)   │  + konturer (hvit)  │
  │  avviste (×)       │                     │
  ├────────────────────┼─────────────────────┤
  │  Blå HSV-maske     │  Avvisnings-logg    │
  │  + konturer        │  (hva ble kastet?)  │
  └────────────────────┴─────────────────────┘

Avvisnings-loggen viser for hvert avvist kontur:
  hvilken gate som slo til + verdien som feilet.

Trykk 'q' for å avslutte.
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import time
import threading
from pathlib import Path

import cv2
import numpy as np

from vision.camera import OAKCamera
from vision.detector import SimpleBallDetector, BallColor, DetectedBall
from config import vision as config

# ── Intern prosesserings-skala (identisk med det detect_balls bruker) ──────────
_DET_SCALE = SimpleBallDetector.DETECTION_SCALE


# ══════════════════════════════════════════════════════════════════════════════
#  Diagnostisk detektor — logger ALLE avviste konturer med grunn
# ══════════════════════════════════════════════════════════════════════════════

class DiagnosticDetector(SimpleBallDetector):
    """
    Utvider SimpleBallDetector med per-frame logging av avviste konturer.

    Etter hvert kall til detect_balls:
      self.rejections  — liste over avviste konturer (gate + verdi)
      self.accepted_raw — liste over godkjente konturer (før tracker, med scores)

    NB: koordinatene i rejections er i HALVSKALA (0.5×) — de skaleres opp
    i build_composite() med ×2 når de tegnes på full-oppløsningsbilde.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._diag_lock   = threading.Lock()
        self.rejections:   list = []   # tømes per frame
        self.accepted_raw: list = []   # godkjente konturer med score-detaljer

    def _validate_contour(self, contour, color, method, hsv=None):
        """
        Delegates to parent SimpleBallDetector._validate_contour and logs
        the result (accepted or rejected) for diagnostic visualization.
        """
        color_name = "RØD" if color == BallColor.RED else "BLÅ"
        (enc_x, enc_y), radius = cv2.minEnclosingCircle(contour)
        approx_center = (int(enc_x), int(enc_y))

        # Call parent implementation
        result = super()._validate_contour(contour, color, method, hsv)

        if result is None:
            # Rejected — log why (simplified: parent doesn't expose reason,
            # so we just log the key metrics for the debug panel)
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0
            bx, by, bw, bh = cv2.boundingRect(contour)
            aspect = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0
            with self._diag_lock:
                self.rejections.append({
                    "reason": f"{color_name}: cir={circularity:.2f} asp={aspect:.2f} r={radius:.0f}",
                    "center": approx_center,
                    "radius": float(radius),
                    "color":  color,
                })
        else:
            # Accepted — log scores
            with self._diag_lock:
                self.accepted_raw.append({
                    "color":  color,
                    "center": result.center,
                    "radius": result.radius,
                    "conf":   result.confidence,
                    "shape":  result.shape_confidence,
                    "color_c": result.color_confidence,
                })

        return result


# ══════════════════════════════════════════════════════════════════════════════
#  Visualisering
# ══════════════════════════════════════════════════════════════════════════════

_C_RED    = (0,   50,  230)
_C_BLUE   = (210, 80,    0)
_C_WHITE  = (255, 255, 255)
_C_GREEN  = (30,  220,  30)
_C_YELLOW = (0,   210, 230)
_C_GRAY   = (140, 140, 140)


def _put_text(img, text, x, y, scale=0.5, fg=_C_WHITE, thickness=1):
    """Tekst med tykk svart skygge for god lesbarhet på alle bakgrunner."""
    cv2.putText(img, text, (x + 1, y + 1),
                cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, scale, fg,        thickness,     cv2.LINE_AA)


def _mask_overlay(mask: np.ndarray, color_bgr: tuple) -> np.ndarray:
    """Konverter binær maske til farget BGR-bilde."""
    out = np.zeros((*mask.shape, 3), dtype=np.uint8)
    out[mask > 0] = color_bgr
    return out


def _stats_panel(balls: list, fps: float, w: int, h: int) -> np.ndarray:
    """Statistikkpanel: deteksjonsoppsummering med konfidenslinjer per ball."""
    panel = np.full((h, w, 3), 14, dtype=np.uint8)

    red_balls  = sorted([b for b in balls if b.color == BallColor.RED],
                        key=lambda b: b.track_id)
    blue_balls = sorted([b for b in balls if b.color == BallColor.BLUE],
                        key=lambda b: b.track_id)

    PAD   = 12
    BAR_X = PAD + 30
    BAR_W = min(w - BAR_X - 44, 84)
    BAR_H = 8
    LH    = 18

    def _bar_row(y, label, color, confidence):
        _put_text(panel, label, PAD, y, scale=0.42, fg=color)
        by_top = y - BAR_H + 1
        cv2.rectangle(panel, (BAR_X, by_top), (BAR_X + BAR_W, by_top + BAR_H),
                      (40, 40, 40), cv2.FILLED)
        fill = max(1, int(BAR_W * confidence))
        cv2.rectangle(panel, (BAR_X, by_top), (BAR_X + fill, by_top + BAR_H),
                      color, cv2.FILLED)
        _put_text(panel, f"{confidence*100:.0f}%",
                  BAR_X + BAR_W + 5, y, scale=0.40, fg=_C_WHITE)

    y = 17
    _put_text(panel, f"Detections:  {len(balls)}",
              PAD, y, scale=0.50, fg=_C_WHITE, thickness=2)
    y += 13
    cv2.line(panel, (PAD, y), (w - PAD, y), (55, 55, 55), 1)
    y += 15

    _put_text(panel, f"Red:  {len(red_balls)}",  PAD, y, scale=0.44, fg=_C_RED,  thickness=2)
    y += LH
    for i, b in enumerate(red_balls, 1):
        _bar_row(y, f"R{i}", _C_RED, b.confidence)
        y += LH
    y += 5
    cv2.line(panel, (PAD, y), (w - PAD, y), (55, 55, 55), 1)
    y += 13

    _put_text(panel, f"Blue: {len(blue_balls)}", PAD, y, scale=0.44, fg=_C_BLUE, thickness=2)
    y += LH
    for i, b in enumerate(blue_balls, 1):
        _bar_row(y, f"B{i}", _C_BLUE, b.confidence)
        y += LH

    _put_text(panel, f"FPS  {fps:.1f}", PAD, h - 10, scale=0.38, fg=_C_GRAY)
    return panel


def build_composite(frame_full:  np.ndarray,
                    red_mask:    np.ndarray,
                    blue_mask:   np.ndarray,
                    balls:       list,
                    rejections:  list,
                    fps:         float) -> np.ndarray:
    """
    Bygg 4-panel composite-bilde.

    Koordinater i `rejections` er halvskala (×_DET_SCALE).
    Vi skalerer dem opp med UP når de tegnes på full-oppløsningsbilde.
    """
    H, W = frame_full.shape[:2]
    PW, PH = W // 2, H // 2
    UP = 1.0 / _DET_SCALE   # oppskaleringsfaktor for _DET_SCALE-koordinater

    def to_panel(img):
        return cv2.resize(img, (PW, PH), interpolation=cv2.INTER_LINEAR)

    # ── Top-Left: råkamera + deteksjoner + avviste ────────────────────────
    tl = frame_full.copy()

    # Avviste konturer: × i fargekoden til ballen
    for r in rejections:
        rc  = (int(r["center"][0] * UP), int(r["center"][1] * UP))
        rr  = max(3, int(r["radius"] * UP))
        col = _C_RED if r["color"] == BallColor.RED else _C_BLUE
        cv2.circle(tl, rc, rr, col, 1)
        cv2.line(tl, (rc[0]-rr, rc[1]-rr), (rc[0]+rr, rc[1]+rr), col, 1)
        cv2.line(tl, (rc[0]+rr, rc[1]-rr), (rc[0]-rr, rc[1]+rr), col, 1)

    # Numberer baller konsistent med statistikkpanelet (sort på track_id)
    red_sorted  = sorted([b for b in balls if b.color == BallColor.RED],
                         key=lambda b: b.track_id)
    blue_sorted = sorted([b for b in balls if b.color == BallColor.BLUE],
                         key=lambda b: b.track_id)
    ball_label = {}
    for i, b in enumerate(red_sorted,  1): ball_label[id(b)] = f"R{i}"
    for i, b in enumerate(blue_sorted, 1): ball_label[id(b)] = f"B{i}"

    # Detekterte baller: sirkel + tallmerke
    for ball in balls:
        bcx, bcy = ball.center
        br  = max(3, int(ball.radius))
        col = _C_RED if ball.color == BallColor.RED else _C_BLUE
        lbl = ball_label.get(id(ball), "?")
        cv2.circle(tl, (bcx, bcy), br, col, 2)
        cv2.circle(tl, (bcx, bcy), 4, col, -1)
        # Liten mørk boks bak etiketten
        (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
        lx, ly = bcx - br, bcy - br - 6
        lx = max(2, lx)
        ly = max(th + 4, ly)
        cv2.rectangle(tl, (lx - 3, ly - th - 2), (lx + tw + 3, ly + 3),
                      (0, 0, 0), cv2.FILLED)
        _put_text(tl, lbl, lx, ly, scale=0.50, fg=col, thickness=1)

    _put_text(tl, f"FPS {fps:.1f}   Baller: {len(balls)}", 6, 24, scale=0.55, fg=_C_WHITE, thickness=2)
    tl = to_panel(tl)

    # ── Top-Right: rød HSV-maske + konturer ──────────────────────────────
    tr = _mask_overlay(red_mask, (70, 70, 255))
    red_cnt, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(tr, red_cnt, -1, _C_WHITE, 1)
    px_r = int(np.sum(red_mask > 0))
    _put_text(tr, f"Rod maske   {px_r} px  ({len(red_cnt)} konturer)", 6, 26,
              scale=0.55, fg=_C_WHITE, thickness=2)
    tr = to_panel(tr)

    # ── Bottom-Left: blå HSV-maske + konturer ────────────────────────────
    bl = _mask_overlay(blue_mask, (220, 110, 20))
    blue_cnt, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(bl, blue_cnt, -1, _C_WHITE, 1)
    px_b = int(np.sum(blue_mask > 0))
    _put_text(bl, f"Bla maske   {px_b} px  ({len(blue_cnt)} konturer)", 6, 26,
              scale=0.55, fg=_C_WHITE, thickness=2)
    bl = to_panel(bl)

    # ── Bottom-Right: statistikkpanel ─────────────────────────────────────
    br = _stats_panel(balls, fps, PW, PH)

    top  = np.hstack([tl, tr])
    bot  = np.hstack([bl, br])
    comp = np.vstack([top, bot])
    cv2.line(comp, (PW, 0),           (PW, comp.shape[0]),  (60, 60, 60), 2)
    cv2.line(comp, (0,  PH),          (comp.shape[1], PH),  (60, 60, 60), 2)
    return comp


# ══════════════════════════════════════════════════════════════════════════════
#  Kamera-loop (kjøres i bakgrunnstråd)
# ══════════════════════════════════════════════════════════════════════════════

def _hsv_masks_from_frame(detector: DiagnosticDetector,
                          frame: np.ndarray) -> tuple:
    """Bygg rød/blå HSV-mask i full oppløsning for visualisering."""
    proc     = cv2.resize(frame, (0, 0), fx=_DET_SCALE, fy=_DET_SCALE)
    lighting = detector.analyze_lighting(proc)
    comp     = detector.apply_lighting_compensation(proc, lighting)
    hsv      = cv2.cvtColor(comp, cv2.COLOR_BGR2HSV)
    red_r, blue_r = detector.red_ranges, detector.blue_ranges

    def build(ranges):
        m = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            m = cv2.bitwise_or(m, cv2.inRange(hsv, lo, hi))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  detector.morph_kernel_small)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, detector.morph_kernel_large)
        # skaler tilbake til full oppløsning
        return cv2.resize(m, (frame.shape[1], frame.shape[0]),
                          interpolation=cv2.INTER_NEAREST)

    return build(red_r), build(blue_r)


def camera_loop(detector: DiagnosticDetector, cam: OAKCamera):
    """Hoved-loop: leser kamera, kjører detektor, viser 4-panel vindu via cv2.imshow."""
    fps    = 0.0
    t_last = time.time()

    cv2.namedWindow("Ball Debug", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Ball Debug", 1280, 800)

    print()
    print("=" * 52)
    print("  Vindu åpnet! Trykk 'q' for å stoppe.")
    print("  TL = kamera + deteksjoner (●) / avviste (×)")
    print("  TR = rød HSV-maske + konturer")
    print("  BL = blå HSV-maske + konturer")
    print("  BR = avvisnings-logg (hva ble kastet og hvorfor)")
    print("=" * 52)

    while True:
        ret, frame = cam.read()
        if not ret or frame is None:
            time.sleep(0.03)
            continue

        # Nullstill diagnostikk-logger for ny frame
        with detector._diag_lock:
            detector.rejections.clear()
            detector.accepted_raw.clear()

        # Kjør detektor
        balls, _ = detector.detect_balls(frame)

        # Bygg HSV-masker for visualisering
        red_mask, blue_mask = _hsv_masks_from_frame(detector, frame)

        # FPS (eksponentielt glattet)
        now    = time.time()
        dt     = max(now - t_last, 1e-5)
        fps    = 0.9 * fps + 0.1 / dt
        t_last = now

        # Snapshot av diagnostikk-logg
        with detector._diag_lock:
            rej_snap = list(detector.rejections)

        # Bygg og vis composite
        composite = build_composite(frame, red_mask, blue_mask,
                                    balls, rej_snap, fps)
        cv2.imshow("Ball Debug", composite)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    cam.release()


# ══════════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    print("Åpner OAK-kamera ...")
    cam = OAKCamera(resolution=config.CAMERA_RESOLUTION)
    if not cam.open():
        print("FEIL: Kunne ikke åpne kameraet")
        sys.exit(1)

    focal_px = cam.get_focal_length_px(hfov_deg=config.CAMERA_HFOV_DEG)
    print(f"Kamera OK   oppløsning={config.CAMERA_RESOLUTION}   f={focal_px:.1f}px")

    detector = DiagnosticDetector(
        min_radius=config.BALL_MIN_RADIUS,
        max_radius=config.BALL_MAX_RADIUS,
        confidence_threshold=config.BALL_CONFIDENCE_THRESHOLD,
        focal_length_px=focal_px,
        max_balls_per_color=10,
    )

    camera_loop(detector, cam)


if __name__ == "__main__":
    main()
