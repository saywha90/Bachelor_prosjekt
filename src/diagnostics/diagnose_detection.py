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


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import cv2
import numpy as np
from pathlib import Path

from vision.camera import OAKCamera
from vision.detector import SimpleBallDetector, BallColor

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
        r = 5
        roi = _hsv_frame_global[max(0, y-r):min(h_fr, y+r+1), max(0, x-r):min(w_fr, x+r+1)]
        if roi.size > 0:
            hh = roi[:, :, 0].reshape(-1)
            ss = roi[:, :, 1].reshape(-1)
            vv = roi[:, :, 2].reshape(-1)
            print(
                f"  [KLIKK]  x={x}, y={y}  →  H={h}  S={s}  V={v}  |  "
                f"11x11 median H/S/V={np.median(hh):.0f}/{np.median(ss):.0f}/{np.median(vv):.0f}  "
                f"S-range={int(ss.min())}-{int(ss.max())}  V-range={int(vv.min())}-{int(vv.max())}"
            )
        else:
            print(f"  [KLIKK]  x={x}, y={y}  →  H={h}  S={s}  V={v}")


def _build_strict_mask(hsv: np.ndarray, ranges) -> np.ndarray:
    """Build only the raw HSV threshold mask before repair/morphology."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    return mask


def _build_combined_mask(
    hsv: np.ndarray,
    ranges,
    detector: "SimpleBallDetector | None" = None,
    color: "BallColor | None" = None,
) -> np.ndarray:
    """Build a colour mask that mirrors the actual detector pipeline.

    For blue, calls detector._repair_blue_mask so specular highlights and
    dim-shadow pixels are recovered (exactly as in production).  Uses the
    same 13×13 closing kernel as SimpleBallDetector.
    """
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))

    # Mirror _build_color_mask: apply blue repair when detector is available.
    if detector is not None and color == BallColor.BLUE:
        mask = detector._repair_blue_mask(hsv, mask)

    kern_s = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kern_l = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))   # match detector
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kern_s)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern_l)

    # Mirror detector._apply_hsv_ranges: fill internal holes after morphology.
    if mask[0, 0] == 0:
        flood = mask.copy()
        ff_mask = np.zeros((mask.shape[0] + 2, mask.shape[1] + 2), dtype=np.uint8)
        cv2.floodFill(flood, ff_mask, (0, 0), 255)
        mask = cv2.bitwise_or(mask, cv2.bitwise_not(flood))
    return mask


def _describe_mask_hsv(label: str, hsv: np.ndarray, mask: np.ndarray) -> None:
    """Print HSV distribution inside a mask to diagnose sunlight washout."""
    px = mask > 0
    count = int(np.sum(px))
    if count == 0:
        print(f"    {label}: 0 px")
        return

    h = hsv[:, :, 0][px]
    s = hsv[:, :, 1][px]
    v = hsv[:, :, 2][px]
    blue_hue = ((h >= 78) & (h <= 135))
    washed = ((s < 40) & (v > 140))
    specular = ((s <= 45) & (v >= 100))
    print(
        f"    {label}: {count} px  "
        f"H p10/50/90={np.percentile(h, 10):.0f}/{np.percentile(h, 50):.0f}/{np.percentile(h, 90):.0f}  "
        f"S p10/50/90={np.percentile(s, 10):.0f}/{np.percentile(s, 50):.0f}/{np.percentile(s, 90):.0f}  "
        f"V p10/50/90={np.percentile(v, 10):.0f}/{np.percentile(v, 50):.0f}/{np.percentile(v, 90):.0f}  "
        f"blue_hue={np.mean(blue_hue)*100:.1f}%  washed(S<40,V>140)={np.mean(washed)*100:.1f}%  "
        f"specular(S<=45,V>=100)={np.mean(specular)*100:.1f}%"
    )


def _print_blue_sunlight_debug(
    frame_n: int,
    hsv: np.ndarray,
    detector: "SimpleBallDetector",
    blue_strict: np.ndarray,
    blue_mask: np.ndarray,
) -> None:
    """Print blue-specific diagnostics for direct-sun/high-glare failures."""
    total_px = hsv.shape[0] * hsv.shape[1]
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    blue_hue = (hue >= 78) & (hue <= 135)
    valid_blue_hue = blue_hue & (sat >= 8) & (val >= 8)
    washed_blue = blue_hue & (sat < 40) & (val > 140)
    bright_low_sat = (sat <= 45) & (val >= 100)
    strict_px = int(np.sum(blue_strict > 0))
    repaired_px = int(np.sum(blue_mask > 0))

    print(f"\n  === BLUE SUNLIGHT DEBUG frame {frame_n} ===")
    print(
        f"    strict seed={strict_px/total_px*100:.3f}%  repaired/final={repaired_px/total_px*100:.3f}%  "
        f"gain={(repaired_px / max(strict_px, 1)):.2f}x"
    )
    print(
        f"    whole-frame blue hue={np.mean(valid_blue_hue)*100:.3f}%  "
        f"washed blue-hue(S<40,V>140)={np.mean(washed_blue)*100:.3f}%  "
        f"bright low-sat glare={np.mean(bright_low_sat)*100:.3f}%"
    )
    _describe_mask_hsv("strict blue seed", hsv, blue_strict)
    _describe_mask_hsv("final blue mask", hsv, blue_mask)

    contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
        print("    largest blue contours:")
        for i, c in enumerate(contours, start=1):
            area = cv2.contourArea(c)
            (x, y), r = cv2.minEnclosingCircle(c)
            ball = detector._validate_contour(c, BallColor.BLUE, "diagnostic", hsv)
            status = "PASS" if ball else "REJECT"
            print(f"      #{i}: area={area:.1f} radius={r:.1f} center=({x:.0f},{y:.0f}) -> {status}")
    else:
        print("    largest blue contours: none")


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


def _contour_rejection_stats(
    mask: np.ndarray,
    min_r: int,
    max_r: int,
    label: str = "",
    color: "BallColor | None" = None,
    hsv: "np.ndarray | None" = None,
):
    """Print why contours fail – uses the same thresholds as _validate_contour.

    For blue the same relaxed gates that were introduced to handle specular/
    crescent masks are applied here so the diagnostic reflects production
    behaviour accurately.
    """
    import math as _math
    is_blue = (color == BallColor.BLUE)

    # Mirror _validate_contour thresholds (non-edge-clipped path).
    min_circ  = 0.55 if is_blue else 0.65
    min_ar    = 0.65 if is_blue else 0.75
    min_sol   = 0.65 if is_blue else 0.75
    min_fill  = 0.60 if is_blue else 0.72
    min_ell   = 0.70 if is_blue else 0.82

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    total = len(contours)
    fail_area = fail_radius = fail_circ = fail_ar = fail_sol = fail_fill = fail_ell = fail_color = 0
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
        if circ < min_circ:
            fail_circ += 1
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        ar = min(bw, bh) / max(bw, bh) if max(bw, bh) > 0 else 0
        if ar < min_ar:
            fail_ar += 1
            continue
        hull = cv2.convexHull(c)
        ha = cv2.contourArea(hull)
        sol = area / ha if ha > 0 else 0
        if sol < min_sol:
            fail_sol += 1
            continue
        fill = area / (_math.pi * r ** 2) if r > 0 else 0
        if fill < min_fill:
            fail_fill += 1
            continue
        ell = 1.0
        if len(c) >= 5:
            try:
                (_ecx, _ecy), (axis_a, axis_b), _angle = cv2.fitEllipse(c)
                ell = min(axis_a, axis_b) / max(axis_a, axis_b) if max(axis_a, axis_b) > 0 else 0
            except cv2.error:
                ell = 0
        if ell < min_ell:
            fail_ell += 1
            continue

        if hsv is not None and color is not None:
            cmask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(cmask, [c], -1, 255, cv2.FILLED)
            h = hsv[:, :, 0][cmask > 0]
            s = hsv[:, :, 1][cmask > 0]
            v = hsv[:, :, 2][cmask > 0]
            if s.size > 0:
                if color == BallColor.BLUE:
                    not_glare = ~((s < 45) & (v > 100))
                    if np.sum(not_glare) > 5:
                        sat_score = float(np.mean(s[not_glare])) / 255.0
                        hue_ratio = float(np.mean((h[not_glare] >= 78) & (h[not_glare] <= 135)))
                    else:
                        sat_score = float(np.mean(s)) / 255.0
                        hue_ratio = float(np.mean((h >= 78) & (h <= 135)))
                    if sat_score < 0.18 or (sat_score < 0.40 and hue_ratio < 0.80):
                        fail_color += 1
                        continue
                elif color == BallColor.RED:
                    not_glare = ~((s <= 90) & (v >= 165))
                    if np.sum(not_glare) > 5:
                        sat_score = float(np.mean(s[not_glare])) / 255.0
                        hue_ratio = float(np.mean((h[not_glare] <= 35) | (h[not_glare] >= 145)))
                    else:
                        sat_score = float(np.mean(s)) / 255.0
                        hue_ratio = float(np.mean((h <= 35) | (h >= 145)))
                    if sat_score < 0.35 or (sat_score < 0.45 and hue_ratio < 0.65):
                        fail_color += 1
                        continue
        passed += 1

    print(f"  [{label}] {total} contours råa  →  "
          f"fail area={fail_area}  radius={fail_radius}  "
          f"circ={fail_circ}  ar={fail_ar}  sol={fail_sol}  fill={fail_fill}  "
          f"ellipse={fail_ell}  color={fail_color}  "
          f"GODKJENT={passed}")


def main():
    global _hsv_frame_global

    detector = SimpleBallDetector(min_radius=10, max_radius=150)

    cam = OAKCamera(resolution=(1280, 720))
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
            # Match production detector: analyze lighting, apply CLAHE when enabled,
            # then build HSV masks from the compensated frame.
            lighting_info = detector.analyze_lighting(frame)
            compensated = detector.apply_lighting_compensation(frame, lighting_info)
            hsv_raw = cv2.cvtColor(compensated, cv2.COLOR_BGR2HSV)
            hsv_blur = hsv_raw

            _hsv_frame_global = hsv_raw  # for mouse callback (no blur — true values)

            # ─── Masker ───────────────────────────────────────────────────────
            red_strict = _build_strict_mask(hsv_blur, detector.red_ranges)
            blue_strict = _build_strict_mask(hsv_blur, detector.blue_ranges)

            red_mask  = _build_combined_mask(hsv_blur, detector.red_ranges,
                                              detector=detector, color=BallColor.RED)
            blue_mask = _build_combined_mask(hsv_blur, detector.blue_ranges,
                                              detector=detector, color=BallColor.BLUE)

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

            put(frame,   f"Frame #{frame_n}  |  klikk for HSV-verdi etter lyskomp", 30)
            if _last_hsv:
                h, s, v = _last_hsv
                put(frame, f"Siste klikk: H={h} S={s} V={v}", 62, (0, 255, 255))

            put(red_vis,  f"ROD MASKE  — {n_red} gyldige contours", 30, (0, 100, 255))
            put(blue_vis, f"BLA MASKE  — {n_blue} gyldige contours", 30, (255, 200, 0))
            put(overlay,  f"Rod={n_red}  Bla={n_blue}  lys={lighting_info['level']} mean={lighting_info['mean_brightness']:.0f}", 30)

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
                      f"contours rød={n_red} blå={n_blue}  |  "
                      f"lys={lighting_info['level']} mean={lighting_info['mean_brightness']:.1f}")
                # Detaljer om HVORFOR contours feiler (bruker samme terskler som _validate_contour)
                _contour_rejection_stats(red_mask,  detector.min_radius, detector.max_radius, "RØD",
                                         color=BallColor.RED, hsv=hsv_blur)
                _contour_rejection_stats(blue_mask, detector.min_radius, detector.max_radius, "BLÅ",
                                         color=BallColor.BLUE, hsv=hsv_blur)
                _print_blue_sunlight_debug(frame_n, hsv_blur, detector, blue_strict, blue_mask)

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
