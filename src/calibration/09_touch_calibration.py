"""
09_touch_calibration.py
========================
Interactive tool to calibrate the homography matrix by physically touching
calibration points. Eliminates ruler measurements entirely and aligns the
vision matrix perfectly with the arm's internal IK coordinate frame.

Accuracy Features
-----------------
- **Auto-detection**: Uses ``SimpleBallDetector`` to find ball centroids
  with sub-pixel accuracy (contour moments), instead of relying on manual
  mouse clicks.
- **Frame averaging**: Averages detected centroids across 30+ frames to
  eliminate single-frame jitter (±1–3 px noise).
- **N-point overdetermined calibration**: Supports 4–9+ calibration points.
  With >4 points, ``cv2.findHomography(RANSAC)`` is used instead of the
  minimum-fit ``cv2.getPerspectiveTransform``, providing least-squares
  averaging and outlier rejection.
- **Reprojection error report**: After computing the homography, the script
  reports per-point and mean reprojection error so you can see how accurate
  the calibration is.

Workflow
--------
  0.  Place N balls (≥ 4) on the workspace in a wide spread.
  1.  Arm moves to SCAN_POSE.
  2.  Camera opens. Balls are auto-detected (or manually clicked as fallback).
  3.  Camera closes.
  4.  For each ball, drive the arm with WASD until the claw is centered over
      the ball, then press ENTER.
  5.  Homography is computed and saved to ``homography_calibration.json``.

Usage
-----
    python -m src.calibration.09_touch_calibration

Author: Bachelor Project 2026 – Autonomia
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import time
from datetime import date
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np

from vision.camera import OAKCamera
from vision.detector import SimpleBallDetector, BallColor
from config import vision as vcfg
from config.arm import SCAN_POSE, SCAN_POSE_TOLERANCE, GRAB_HEIGHT, CLEARANCE_HEIGHT
from ik.solver import ArmIK

# ── Constants ─────────────────────────────────────────────────────────
MIN_POINTS = 4
MAX_POINTS = 12
AVG_FRAMES = 40            # number of frames to average for centroid
DETECTION_SETTLE = 10      # discard first N frames (camera auto-exposure)
WINDOW_NAME = "Touch Calibration"
CALIBRATION_FILE = Path(__file__).resolve().parent / "homography_calibration.json"

SERIAL_PORT = "/dev/cu.usbmodem2101"
SERIAL_BAUD = 115200

_ser = None

# ── Manual-click state (fallback) ─────────────────────────────────────
_clicked_points: List[Tuple[int, int]] = []
_current_mouse = (0, 0)
_click_limit = 4


# ══════════════════════════════════════════════════════════════════════
#  Serial / Arm helpers
# ══════════════════════════════════════════════════════════════════════

def _get_serial():
    global _ser
    if _ser is None:
        import serial
        print(f"[SERIAL] Opening {SERIAL_PORT} @ {SERIAL_BAUD} …")
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=2)
        time.sleep(3)
        boot_msg = ""
        while _ser.in_waiting:
            boot_msg += _ser.readline().decode(errors="replace").strip() + " "
        if not boot_msg.strip():
            boot_msg = _ser.readline().decode(errors="replace").strip()
        print(f"[SERIAL] OpenRB says: {boot_msg.strip()}")

        cmd = json.dumps({"cmd": "enable_torque"})
        _ser.write((cmd + "\n").encode())
        _ser.readline()

        cmd = json.dumps({"cmd": "set_profile", "vel": 40, "acc": 10})
        _ser.write((cmd + "\n").encode())
        _ser.readline()
    return _ser


def send_raw_command(positions: dict):
    ser = _get_serial()
    cmd_json = json.dumps(positions)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    return resp


def send_ik_command(arm: ArmIK, x: float, y: float, z: float,
                    claw_override: int = 2048):
    ser = _get_serial()
    solution = arm.solve(x, y, z)
    solution["m5"] = claw_override
    cmd_json = json.dumps(solution)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    return resp


def _move_to_scan_pose():
    print("[ARM] Moving arm to SCAN_POSE for calibration...")
    send_raw_command(SCAN_POSE)
    time.sleep(2)
    print("[ARM] ✅ Arm is at SCAN_POSE.\n")


# ══════════════════════════════════════════════════════════════════════
#  Phase 1 — Auto-detect ball centroids with frame averaging
# ══════════════════════════════════════════════════════════════════════

def _auto_detect_balls(cam: OAKCamera) -> List[Tuple[float, float]]:
    """Detect all balls in the camera feed, average centroids over many frames.

    Returns a list of (cx, cy) pixel coordinates sorted left-to-right.
    """
    detector = SimpleBallDetector(
        min_radius=vcfg.BALL_MIN_RADIUS,
        max_radius=vcfg.BALL_MAX_RADIUS,
        confidence_threshold=max(vcfg.BALL_CONFIDENCE_THRESHOLD, 0.3),
        enable_adaptive_lighting=True,
        max_balls_per_color=6,
    )

    # Let camera auto-exposure settle
    print(f"  ⏳  Letting camera settle ({DETECTION_SETTLE} frames)...")
    for _ in range(DETECTION_SETTLE):
        cam.read()

    # Accumulate detections over AVG_FRAMES
    print(f"  ⏳  Averaging ball positions over {AVG_FRAMES} frames...")
    all_centroids: dict[int, List[Tuple[float, float]]] = {}

    last_frame = None
    for frame_idx in range(AVG_FRAMES):
        ret, frame = cam.read()
        if not ret or frame is None:
            continue
        last_frame = frame.copy()

        balls, _ = detector.detect_balls(frame)
        for ball in balls:
            if ball.color == BallColor.UNKNOWN:
                continue
            cx, cy = ball.center
            # Assign to nearest existing cluster or create new one
            matched = False
            for cid, centroid_list in all_centroids.items():
                avg_x = np.mean([c[0] for c in centroid_list])
                avg_y = np.mean([c[1] for c in centroid_list])
                if abs(cx - avg_x) < 40 and abs(cy - avg_y) < 40:
                    centroid_list.append((float(cx), float(cy)))
                    matched = True
                    break
            if not matched:
                all_centroids[len(all_centroids)] = [(float(cx), float(cy))]

    # Average each cluster
    averaged: List[Tuple[float, float]] = []
    for cid, centroid_list in all_centroids.items():
        if len(centroid_list) < AVG_FRAMES * 0.3:
            # Ball was detected in < 30% of frames — unreliable, skip
            continue
        avg_cx = float(np.mean([c[0] for c in centroid_list]))
        avg_cy = float(np.mean([c[1] for c in centroid_list]))
        n = len(centroid_list)
        std_x = float(np.std([c[0] for c in centroid_list]))
        std_y = float(np.std([c[1] for c in centroid_list]))
        print(f"       Ball {len(averaged)+1}: ({avg_cx:.1f}, {avg_cy:.1f}) px  "
              f"[{n}/{AVG_FRAMES} frames, σ=({std_x:.1f}, {std_y:.1f}) px]")
        averaged.append((avg_cx, avg_cy))

    return averaged, last_frame


def _preview_detections(frame: np.ndarray,
                        points: List[Tuple[float, float]]) -> bool:
    """Show detected ball positions on the last frame and ask user to confirm.

    Returns True if user accepts, False if they want to fall back to manual.
    """
    overlay = frame.copy()
    h, w = overlay.shape[:2]

    cv2.rectangle(overlay, (0, 0), (w, 36), (0, 80, 0), -1)
    cv2.putText(overlay,
                f"Auto-detected {len(points)} balls.  ENTER=accept  R=retry  M=manual click",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    for i, (cx, cy) in enumerate(points):
        px, py = int(round(cx)), int(round(cy))
        cv2.circle(overlay, (px, py), 10, (0, 0, 0), 2)
        cv2.circle(overlay, (px, py), 8, (0, 255, 0), 2)
        cv2.putText(overlay, f"#{i+1}", (px + 14, py + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    cv2.namedWindow(WINDOW_NAME)
    cv2.imshow(WINDOW_NAME, overlay)

    while True:
        key = cv2.waitKey(0) & 0xFF
        if key == 13:  # ENTER
            cv2.destroyAllWindows()
            return True
        elif key in (ord('m'), ord('M')):
            cv2.destroyAllWindows()
            return False
        elif key in (ord('r'), ord('R')):
            cv2.destroyAllWindows()
            return None  # type: ignore[return-value]  # signals retry
        elif key in (ord('q'), ord('Q')):
            cv2.destroyAllWindows()
            sys.exit(0)


# ══════════════════════════════════════════════════════════════════════
#  Phase 1 fallback — Manual clicking
# ══════════════════════════════════════════════════════════════════════

def _mouse_callback(event, x, y, flags, param):
    global _current_mouse
    _current_mouse = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(_clicked_points) < _click_limit:
            _clicked_points.append((x, y))
            print(f"  ✅  Clicked point #{len(_clicked_points)} at ({x}, {y})")


def _draw_click_overlay(frame: np.ndarray,
                        points: List[Tuple[int, int]],
                        mouse: Tuple[int, int],
                        n_target: int) -> np.ndarray:
    overlay = frame.copy()
    h, w = overlay.shape[:2]

    n_done = len(points)
    if n_done < n_target:
        label = f"Click ball {n_done+1}/{n_target}  (R=reset, Q=quit)"
        cv2.rectangle(overlay, (0, 0), (w, 36), (0, 0, 0), -1)
        cv2.putText(overlay, label, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2, cv2.LINE_AA)
    else:
        cv2.rectangle(overlay, (0, 0), (w, 36), (0, 80, 0), -1)
        cv2.putText(overlay, f"All {n_target} points clicked! Press ENTER to continue or R to reset.",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    for i, (px, py) in enumerate(points):
        cv2.circle(overlay, (px, py), 8, (0, 0, 0), -1)
        cv2.circle(overlay, (px, py), 6, (0, 255, 0), -1)
        cv2.putText(overlay, f"#{i+1}", (px + 12, py + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

    # Draw quadrilateral edges between consecutive points
    for i in range(len(points)):
        j = (i + 1) % len(points)
        if j < len(points) and len(points) > 1:
            cv2.line(overlay, points[i], points[j], (100, 100, 100), 1)

    # Crosshair cursor
    mx, my = mouse
    cv2.line(overlay, (mx - 15, my), (mx + 15, my), (255, 255, 255), 1)
    cv2.line(overlay, (mx, my - 15), (mx, my + 15), (255, 255, 255), 1)
    return overlay


def _manual_click_phase(cam: OAKCamera, n_target: int) -> List[Tuple[float, float]]:
    """Fall back to manual clicking when auto-detection can't find enough balls."""
    global _clicked_points, _click_limit
    _clicked_points.clear()
    _click_limit = n_target

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, _mouse_callback)

    print(f"\n  🖱️  Click {n_target} balls in the camera feed:")

    try:
        while True:
            ret, frame = cam.read()
            if not ret or frame is None:
                continue

            overlay = _draw_click_overlay(frame, _clicked_points, _current_mouse, n_target)
            cv2.imshow(WINDOW_NAME, overlay)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), ord('Q')):
                cv2.destroyAllWindows()
                sys.exit(0)
            if key == ord('r'):
                _clicked_points.clear()
                print("  🔄  Reset — click again")
            if len(_clicked_points) >= n_target:
                # Wait for ENTER to confirm or R to reset
                if key == 13:
                    break
    except KeyboardInterrupt:
        cam.release()
        cv2.destroyAllWindows()
        sys.exit(0)

    points = [(float(x), float(y)) for x, y in _clicked_points]
    cv2.destroyAllWindows()
    return points


# ══════════════════════════════════════════════════════════════════════
#  Phase 2 — Physical touch
# ══════════════════════════════════════════════════════════════════════

def _touch_phase(arm: ArmIK,
                 pixel_points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Drive the arm to each ball and record IK coordinates."""
    cm_points: List[Tuple[float, float]] = []
    n = len(pixel_points)

    cv2.namedWindow(WINDOW_NAME)

    # Initial safe starting coordinates
    target_x = 25.0
    target_y = 0.0
    target_z = float(GRAB_HEIGHT)
    step = 1.0

    print(f"\n{'─'*60}")
    print("  🕹️  PHYSICAL TOUCH PHASE")
    print(f"       Drive the arm to each of the {n} balls.")
    print("       Center the OPEN CLAW exactly over each ball.")
    print("       W / S = Move Forward / Back (X)")
    print("       A / D = Move Left / Right (Y)")
    print("       U / J = Move Up / Down (Z)")
    print("       [ / ] = Change Step Size (currently 1.0 cm)")
    print("       ENTER = Save Point")
    print(f"{'─'*60}")

    for i in range(n):
        label = f"Ball #{i+1}/{n}"
        px, py = pixel_points[i]
        print(f"\n  👉  Target: {label}  (pixel {px:.0f}, {py:.0f})")

        # Lift to clearance, then down to target_z
        send_ik_command(arm, target_x, target_y, CLEARANCE_HEIGHT)
        time.sleep(0.5)
        send_ik_command(arm, target_x, target_y, target_z)
        time.sleep(0.5)

        while True:
            # Create a black HUD screen for controls
            hud = np.zeros((400, 600, 3), dtype=np.uint8)
            cv2.putText(hud, f"Target: {label}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(hud, f"X = {target_x:.1f} cm  (forward/back)",
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.putText(hud, f"Y = {target_y:.1f} cm  (left/right)",
                        (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.putText(hud, f"Z = {target_z:.1f} cm  (up/down)",
                        (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.putText(hud, f"Step Size = {step:.2f} cm",
                        (20, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
            cv2.putText(hud, "W/S: X   A/D: Y   U/J: Z",
                        (20, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(hud, "[/]: Step   ENTER: Save",
                        (20, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(hud, "TIP: Use [ to reduce step to 0.10 cm for fine alignment",
                        (20, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

            cv2.imshow(WINDOW_NAME, hud)
            key = cv2.waitKey(0) & 0xFF

            moved = False
            if key in (ord('w'), ord('W')):
                target_x += step; moved = True
            elif key in (ord('s'), ord('S')):
                target_x -= step; moved = True
            elif key in (ord('a'), ord('A')):
                target_y += step; moved = True
            elif key in (ord('d'), ord('D')):
                target_y -= step; moved = True
            elif key in (ord('u'), ord('U')):
                target_z += step; moved = True
            elif key in (ord('j'), ord('J')):
                target_z -= step; moved = True
            elif key == ord('['):
                step = max(0.10, step / 2)
                print(f"       Step → {step:.2f} cm")
            elif key == ord(']'):
                step = min(5.0, step * 2)
                print(f"       Step → {step:.2f} cm")
            elif key == 13:  # ENTER
                print(f"       ✅ Saved {label} at ({target_x:.1f}, {target_y:.1f}) cm")
                cm_points.append((target_x, target_y))
                break
            elif key in (ord('q'), ord('Q')):
                print("\n  ⛔ Quit requested.")
                sys.exit(0)

            if moved:
                send_ik_command(arm, target_x, target_y, target_z)

        # Lift before going to next
        send_ik_command(arm, target_x, target_y, CLEARANCE_HEIGHT)
        time.sleep(0.5)

    cv2.destroyAllWindows()
    return cm_points


# ══════════════════════════════════════════════════════════════════════
#  Phase 3 — Compute homography + reprojection error
# ══════════════════════════════════════════════════════════════════════

def _compute_homography(pixel_points: List[Tuple[float, float]],
                        cm_points: List[Tuple[float, float]]):
    """Compute the homography and print a reprojection error report.

    Uses ``cv2.getPerspectiveTransform`` for exactly 4 points, or
    ``cv2.findHomography(RANSAC)`` for 5+ points.
    """
    px_array = np.float32(pixel_points)
    cm_array = np.float32(cm_points)
    n = len(pixel_points)

    if n == 4:
        homography = cv2.getPerspectiveTransform(px_array, cm_array)
        method_name = "getPerspectiveTransform (exact 4-point)"
        inlier_mask = np.ones(n, dtype=bool)
    else:
        homography, mask = cv2.findHomography(px_array, cm_array,
                                              cv2.RANSAC, 3.0)
        inlier_mask = mask.ravel().astype(bool) if mask is not None else np.ones(n, dtype=bool)
        n_inliers = int(inlier_mask.sum())
        n_outliers = n - n_inliers
        method_name = f"findHomography RANSAC ({n_inliers} inliers, {n_outliers} outliers)"

    # ── Reprojection error report ─────────────────────────────────────
    print(f"\n{'═'*60}")
    print(f"  HOMOGRAPHY RESULT  ({method_name})")
    print(f"{'═'*60}")

    errors = []
    for i in range(n):
        px_h = np.float64([[pixel_points[i][0], pixel_points[i][1], 1.0]])
        projected = (homography @ px_h.T).T
        projected = projected[0, :2] / projected[0, 2]
        actual = np.float64([cm_points[i][0], cm_points[i][1]])
        err = float(np.linalg.norm(projected - actual))
        errors.append(err)

        status = "✅" if inlier_mask[i] else "❌ outlier"
        print(f"    Point #{i+1}: pixel ({pixel_points[i][0]:.0f}, {pixel_points[i][1]:.0f}) "
              f"→ expected ({cm_points[i][0]:.1f}, {cm_points[i][1]:.1f}) cm "
              f"→ got ({projected[0]:.1f}, {projected[1]:.1f}) cm "
              f"→ error {err:.2f} cm  {status}")

    mean_err = float(np.mean(errors))
    max_err = float(np.max(errors))
    print(f"\n    Mean reprojection error: {mean_err:.3f} cm")
    print(f"    Max  reprojection error: {max_err:.3f} cm")

    if n == 4:
        print("    ℹ️  With exactly 4 points, error is always ~0 (exact fit).")
        print("    💡 Use 6+ balls for a robust calibration with error averaging.")
    elif mean_err < 0.5:
        print("    ✅ Excellent calibration!")
    elif mean_err < 1.0:
        print("    ✅ Good calibration.")
    elif mean_err < 2.0:
        print("    ⚠️  Moderate calibration — consider re-touching inaccurate points.")
    else:
        print("    ❌ Poor calibration — redo with more care on the touch phase.")

    print(f"{'═'*60}")
    return homography


# ══════════════════════════════════════════════════════════════════════
#  Save calibration
# ══════════════════════════════════════════════════════════════════════

def _save_calibration(pixel_points, cm_points, homography):
    data = {
        "calibrated_at_scan_pose": {k: int(v) for k, v in SCAN_POSE.items()},
        "tolerance": int(SCAN_POSE_TOLERANCE),
        "workspace_px": [[round(x, 1), round(y, 1)] for x, y in pixel_points],
        "workspace_cm": [[round(x, 2), round(y, 2)] for x, y in cm_points],
        "homography": homography.tolist(),
        "calibration_date": date.today().isoformat(),
        "n_calibration_points": len(pixel_points),
    }
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  💾  Calibration saved to {CALIBRATION_FILE}")


# ══════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║     TOUCH CALIBRATION v2 — Auto-Detect + Multi-Point        ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  Place 4–9 balls on the workspace, spread as wide as        ║")
    print("║  possible. More balls = more accurate calibration.          ║")
    print("║                                                              ║")
    print("║  Step 1: Arm moves to SCAN_POSE.                            ║")
    print("║  Step 2: Balls are auto-detected (or click manually).       ║")
    print("║  Step 3: Drive the arm to touch each ball (WASD + ENTER).   ║")
    print("║  Step 4: Homography computed with error report.             ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    # Ask how many balls
    while True:
        n_input = input(f"How many balls are on the desk? [{MIN_POINTS}–{MAX_POINTS}] (default 6): ").strip()
        if not n_input:
            n_balls = 6
            break
        try:
            n_balls = int(n_input)
            if MIN_POINTS <= n_balls <= MAX_POINTS:
                break
            print(f"  ⚠️  Must be between {MIN_POINTS} and {MAX_POINTS}.")
        except ValueError:
            print("  ⚠️  Enter a number.")

    input(f"\nPress ENTER when {n_balls} balls are placed on the desk...")

    # ── Connect arm and move to SCAN_POSE ─────────────────────────────
    try:
        _move_to_scan_pose()
    except Exception as e:
        print(f"❌ Could not connect: {e}")
        return

    arm = ArmIK()

    # ── Open camera ───────────────────────────────────────────────────
    print("[INIT] Opening OAK-D camera...")
    cam = OAKCamera(resolution=vcfg.CAMERA_RESOLUTION)
    if not cam.open():
        print("❌ Could not open camera.")
        return

    # ── Phase 1: Detect ball positions ────────────────────────────────
    pixel_points: Optional[List[Tuple[float, float]]] = None
    use_auto = True

    while pixel_points is None:
        if use_auto:
            print(f"\n  🔍  Auto-detecting {n_balls} balls...")
            detected, last_frame = _auto_detect_balls(cam)

            if len(detected) < MIN_POINTS:
                print(f"  ⚠️  Only found {len(detected)} balls (need at least {MIN_POINTS}).")
                print("       Falling back to manual clicking.\n")
                use_auto = False
                continue

            if len(detected) != n_balls:
                print(f"  ⚠️  Found {len(detected)} balls but expected {n_balls}.")
                print(f"       Proceeding with {len(detected)} detected balls.\n")

            # Show preview for confirmation
            if last_frame is not None:
                result = _preview_detections(last_frame, detected)
                if result is True:
                    pixel_points = detected
                elif result is False:
                    use_auto = False
                    continue
                else:
                    # Retry
                    continue
            else:
                pixel_points = detected
        else:
            pixel_points = _manual_click_phase(cam, n_balls)

    cam.release()
    time.sleep(0.5)

    n_actual = len(pixel_points)
    print(f"\n  ✅  {n_actual} pixel positions captured.")

    # ── Phase 2: Touch Phase ──────────────────────────────────────────
    cm_points = _touch_phase(arm, pixel_points)

    # ── Phase 3: Compute Homography ───────────────────────────────────
    homography = _compute_homography(pixel_points, cm_points)

    # ── Save ──────────────────────────────────────────────────────────
    _save_calibration(pixel_points, cm_points, homography)

    # Return to scan pose when done
    _move_to_scan_pose()
    print("\n  Done. You can now run main.py! ✅\n")


if __name__ == "__main__":
    main()
