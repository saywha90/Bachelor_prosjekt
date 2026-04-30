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
- **N-point overdetermined calibration**: Supports 4–20 calibration points.
  With >4 points, ``cv2.findHomography(RANSAC)`` is used instead of the
  minimum-fit ``cv2.getPerspectiveTransform``, providing least-squares
  averaging and outlier rejection.
- **Reprojection error report**: After computing the homography, the script
  reports per-point and mean reprojection error so you can see how accurate
  the calibration is.

Alternative approaches considered: ArUco markers (rejected due to print/placement
requirements), ruler-based measurement (replaced by this touch method). See ADR-004.

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
import math
import time
from datetime import date
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np

from vision.camera import OAKCamera
from vision.detector import SimpleBallDetector, BallColor
from config import vision as vcfg
from config.arm import (
    SCAN_POSE, SCAN_POSE_TOLERANCE, GRAB_HEIGHT, CLEARANCE_HEIGHT,
    compute_grab_height, compute_wrist_correction,
    M3_SCAN_CURRENT_LIMIT, M3_DEFAULT_CURRENT_LIMIT, HOME_POSITION
)
from ik.solver import ArmIK

# ── Constants ─────────────────────────────────────────────────────────
MIN_POINTS = 4
MAX_POINTS = 20
AVG_FRAMES = 40            # number of frames to average for centroid
DETECTION_SETTLE = 10      # discard first N frames (camera auto-exposure)
WINDOW_NAME = "Touch Calibration"
CALIBRATION_FILE = Path(__file__).resolve().parent / "homography_calibration.json"

SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD = 115200

# ── Manual-click state (fallback) ─────────────────────────────────────
_clicked_points: List[Tuple[int, int]] = []
_current_mouse = (0, 0)
_click_limit = 4


# ══════════════════════════════════════════════════════════════════════
#  Serial / Arm helpers
# ══════════════════════════════════════════════════════════════════════

def _open_serial():
    """Open a new serial connection to the arm and return it.

    This is called once in ``main()``; the returned object is passed as a
    parameter to every function that needs to talk to hardware.  Importing
    this module does **not** open a serial port.
    """
    import serial
    print(f"[SERIAL] Opening {SERIAL_PORT} @ {SERIAL_BAUD} …")
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=2)
    time.sleep(3)
    boot_msg = ""
    while ser.in_waiting:
        boot_msg += ser.readline().decode(errors="replace").strip() + " "
    if not boot_msg.strip():
        boot_msg = ser.readline().decode(errors="replace").strip()
    print(f"[SERIAL] OpenRB says: {boot_msg.strip()}")

    cmd = json.dumps({"cmd": "enable_torque"})
    ser.write((cmd + "\n").encode())
    ser.readline()

    cmd = json.dumps({"cmd": "set_profile", "vel": 40, "acc": 10})
    ser.write((cmd + "\n").encode())
    ser.readline()
    return ser


def send_raw_command(ser, positions: dict):
    cmd_json = json.dumps(positions)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    return resp


def send_ik_command(ser, arm: ArmIK, x: float, y: float, z: float,
                    claw_override: int = 2048):
    try:
        solution = arm.solve(x, y, z, skip_sag=True, strict=True)
    except ValueError as e:
        print(f"  ⚠️  Target unreachable: {e}")
        return None
    solution["m5"] = claw_override
    cmd_json = json.dumps(solution)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    return resp


def send_ik_with_m4_offset(ser, arm: ArmIK, x: float, y: float, z: float,
                           m4_offset: int = 0, claw_override: int = 2048):
    """Solve IK, apply an m4 offset, clamp, send command, and return (solution, response).

    Parameters
    ----------
    ser
        Open serial connection to the arm.
    arm : ArmIK
        The IK solver instance.
    x, y, z : float
        Target position in arm-frame centimetres.
    m4_offset : int
        Dynamixel step offset to add to the IK-computed m4.
    claw_override : int
        Claw position (m5) override.

    Returns
    -------
    tuple[dict, str]
        (solution_dict, firmware_response)
    """
    try:
        solution = arm.solve(x, y, z, skip_sag=True, strict=True)
    except ValueError as e:
        print(f"  ⚠️  Target unreachable: {e}")
        return None, None
    solution["m4"] = max(500, min(3500, solution["m4"] + m4_offset))
    solution["m5"] = claw_override
    cmd_json = json.dumps(solution)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    return solution, resp


def _move_to_scan_pose(ser):
    print("[ARM] Moving arm to SCAN_POSE for calibration...")
    send_raw_command(ser, SCAN_POSE)
    time.sleep(2)
    if M3_SCAN_CURRENT_LIMIT > 0:
        print(f"[ARM] Applying M3 thermal limit ({M3_SCAN_CURRENT_LIMIT} mA)")
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 3, "value": M3_SCAN_CURRENT_LIMIT})
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
        confidence_threshold=0.20,           # lowered for calibration — catch all balls
        enable_adaptive_lighting=True,
        max_balls_per_color=MAX_POINTS,       # allow up to MAX_POINTS per color
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
            # Threshold = max expected pixel jitter for the same physical ball
            # between frames, NOT contour filter size.  2*BALL_MAX_RADIUS (300px)
            # merged distinct balls that were only ~200px apart.
            cluster_threshold = max(3 * vcfg.BALL_MIN_RADIUS, 40)
            matched = False
            for cid, centroid_list in all_centroids.items():
                avg_x = np.mean([c[0] for c in centroid_list])
                avg_y = np.mean([c[1] for c in centroid_list])
                dist = math.hypot(cx - avg_x, cy - avg_y)
                if dist < cluster_threshold:
                    if dist > 0.7 * cluster_threshold:
                        print(f"  ⚠️  Ball at ({cx:.0f}, {cy:.0f}) absorbed into "
                              f"cluster at ({avg_x:.0f}, {avg_y:.0f}) — check ball spacing")
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


def load_previous_homography() -> Optional[np.ndarray]:
    try:
        if CALIBRATION_FILE.exists():
            with open(CALIBRATION_FILE, "r") as f:
                data = json.load(f)
                return np.array(data["homography"], dtype=np.float64)
    except Exception:
        pass
    return None


# ══════════════════════════════════════════════════════════════════════
#  Phase 2 — Physical touch
# ══════════════════════════════════════════════════════════════════════

def _touch_phase(ser, arm: ArmIK,
                 pixel_points: List[Tuple[float, float]],
                 coarse_cm_points: Optional[List[Tuple[float, float, float, int]]] = None) -> List[Tuple[float, float, float, int]]:
    """Drive the arm to each ball and record IK coordinates (x, y, z, m4_offset).

    Parameters
    ----------
    ser
        Open serial connection to the arm.
    arm : ArmIK
        The IK solver instance.
    pixel_points : list
        Detected ball positions in pixel coordinates.
    coarse_cm_points : list, optional
        Pre-recorded coarse positions from limp mode for fine-tuning.

    The user can fine-tune the wrist tilt (m4) with I/K keys in addition to
    the standard WASD/UJ movement.  The learned m4 offset is saved per point
    and later used for runtime wrist correction.
    """
    if M3_DEFAULT_CURRENT_LIMIT > 0:
        print(f"\n[ARM] Restoring M3 full power for manual driving ({M3_DEFAULT_CURRENT_LIMIT} mA)")
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 3, "value": M3_DEFAULT_CURRENT_LIMIT})

    cm_points: List[Tuple[float, float, float, int]] = []
    n = len(pixel_points)

    cv2.namedWindow(WINDOW_NAME)

    prev_homography = load_previous_homography()
    if prev_homography is not None:
        print("\n  💡 Using previous calibration to auto-drive close to targets!")

    # Initial safe starting coordinates
    target_x = 25.0
    target_y = 0.0
    target_z = 16.5
    step = 0.5 if prev_homography is not None else 1.0
    step_m4 = 10          # Dynamixel steps per I/K press (~0.88°)
    m4_offset = 0         # cumulative wrist offset (resets per ball)

    print(f"\n{'─'*60}")
    print("  🕹️  PHYSICAL TOUCH PHASE")
    print(f"       Drive the arm to each of the {n} balls.")
    print("       Center the OPEN CLAW exactly over each ball.")
    print("       W / S = Move Forward / Back (X)")
    print("       A / D = Move Left / Right (Y)")
    print("       U / J = Move Up / Down (Z)")
    print("       I / K = Tilt Wrist Forward / Back (m4)")
    print("       [ / ] = Change Step Size (currently 1.0 cm)")
    print("       ENTER = Save Point")
    print(f"{'─'*60}")

    for i in range(n):
        label = f"Ball #{i+1}/{n}"
        px, py = pixel_points[i]
        
        if coarse_cm_points is not None:
            target_x, target_y, target_z, m4_offset = coarse_cm_points[i]
            step = 0.10  # Fine tuning step size
            print(f"\n  👉  Target: {label}  (pixel {px:.0f}, {py:.0f})")
            print(f"       Auto-driving to coarse point: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")
        else:
            if prev_homography is not None:
                pt = np.float32([[[px, py]]])
                transformed = cv2.perspectiveTransform(pt, prev_homography)
                target_x = float(transformed[0, 0, 0])
                target_y = float(transformed[0, 0, 1])
                target_x = max(10.0, min(75.0, target_x))
                target_y = max(-30.0, min(30.0, target_y))
                # Start high to avoid crashing into things if the guess is wrong
                target_z = 16.5
                m4_offset = compute_wrist_correction(target_x, target_y)
            else:
                m4_offset = 0     # reset wrist offset for each new ball
                
            print(f"\n  👉  Target: {label}  (pixel {px:.0f}, {py:.0f})")
            if prev_homography is not None:
                print(f"       Auto-driving to guess: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")

        # Slow down profile for smooth automated moves
        send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})

        # Move to clearance height above the new target with NO wrist offset.
        # (Pre-tilting the wrist while the arm is fully extended at clearance height 
        # causes the claw to point up and crash into the elbow bracket)
        send_ik_command(ser, arm, target_x, target_y, CLEARANCE_HEIGHT)
        time.sleep(0.5)
        
        # Lower down and apply the wrist offset gently
        send_ik_with_m4_offset(ser, arm, target_x, target_y, target_z, m4_offset)
        time.sleep(0.5)
        
        # Restore normal velocity for WASD manual driving
        send_raw_command(ser, {"cmd": "set_profile", "vel": 80, "acc": 20})

        while True:
            # Create a black HUD screen for controls
            hud = np.zeros((450, 600, 3), dtype=np.uint8)
            cv2.putText(hud, f"Target: {label}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(hud, f"X = {target_x:.1f} cm  (forward/back)",
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.putText(hud, f"Y = {target_y:.1f} cm  (left/right)",
                        (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cv2.putText(hud, f"Z = {target_z:.1f} cm  (up/down)",
                        (20, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            if target_z <= arm.Z_MIN:
                cv2.putText(hud, "Z at minimum! (IK clamp)",
                            (320, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            cv2.putText(hud, f"m4 offset = {m4_offset:+d} steps  (wrist tilt)",
                        (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 1)
            cv2.putText(hud, f"Step Size = {step:.2f} cm",
                        (20, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
            cv2.putText(hud, "W/S: X   A/D: Y   U/J: Z   I/K: Wrist",
                        (20, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(hud, "[/]: Step   ENTER: Save",
                        (20, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(hud, "TIP: Use I/K to fine-tune wrist tilt at far distances",
                        (20, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

            cv2.imshow(WINDOW_NAME, hud)
            key = cv2.waitKey(0) & 0xFF

            moved = False
            wrist_moved = False
            
            old_x, old_y, old_z = target_x, target_y, target_z
            old_m4 = m4_offset

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
            elif key in (ord('i'), ord('I')):
                m4_offset += step_m4; wrist_moved = True
                print(f"       m4 offset → {m4_offset:+d}")
            elif key in (ord('k'), ord('K')):
                m4_offset -= step_m4; wrist_moved = True
                print(f"       m4 offset → {m4_offset:+d}")
            elif key == ord('['):
                step = max(0.10, step / 2)
                print(f"       Step → {step:.2f} cm")
            elif key == ord(']'):
                step = min(5.0, step * 2)
                print(f"       Step → {step:.2f} cm")
            elif key == 13:  # ENTER
                print(f"       ✅ Saved {label} at ({target_x:.1f}, {target_y:.1f}, "
                      f"z={target_z:.1f}) cm, m4_offset={m4_offset:+d}")
                cm_points.append((target_x, target_y, target_z, m4_offset))
                break
            elif key in (ord('q'), ord('Q')):
                print("\n  ⛔ Quit requested.")
                sys.exit(0)

            if moved or wrist_moved:
                solution, resp = send_ik_with_m4_offset(ser, arm, target_x, target_y, target_z, m4_offset)
                if solution is None:
                    # Revert to previous safe values
                    target_x, target_y, target_z = old_x, old_y, old_z
                    m4_offset = old_m4
                    print("       ⚠️  Move reverted (unreachable).")

        # Slow down profile for a safe, smooth retraction
        send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})

        # Lift straight up with NO wrist offset to avoid crashing claw into elbow
        send_ik_command(ser, arm, target_x, target_y, CLEARANCE_HEIGHT)
        time.sleep(0.5)
        
        # Retract to home position to prevent swinging the claw across the desk
        send_ik_command(ser, arm, HOME_POSITION[0], HOME_POSITION[1], HOME_POSITION[2])
        time.sleep(0.5)

    cv2.destroyAllWindows()
    return cm_points


# ══════════════════════════════════════════════════════════════════════
#  Phase 2b — Limp-mode (manual) touch
# ══════════════════════════════════════════════════════════════════════

def _limp_touch_phase(ser, arm: ArmIK,
                      pixel_points: List[Tuple[float, float]]) -> List[Tuple[float, float, float, int]]:
    """Record calibration points by manually guiding the limp arm.

    Instead of using WASD keyboard control with active motors, this
    disables torque on motors 1–4 so the user can physically move the
    arm by hand to each ball position.  Motor 5 (claw) stays torqued
    to hold its position.

    Forward kinematics converts the read motor positions to (x, y, z,
    m4_offset) — the same format that :func:`_touch_phase` produces.

    Parameters
    ----------
    ser
        Open serial connection to the arm.
    arm : ArmIK
        The IK solver instance (used for forward kinematics).
    pixel_points : list
        Detected ball positions in pixel coordinates.

    Returns
    -------
    list[tuple[float, float, float, int]]
        Recorded (x, y, z, m4_offset) for each ball.
    """
    if M3_DEFAULT_CURRENT_LIMIT > 0:
        print(f"\n[ARM] Restoring M3 full power ({M3_DEFAULT_CURRENT_LIMIT} mA)")
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 3, "value": M3_DEFAULT_CURRENT_LIMIT})

    cm_points: List[Tuple[float, float, float, int]] = []
    n = len(pixel_points)

    print(f"\n{'═'*60}")
    print("  🖐️  LIMP-MODE TOUCH PHASE")
    print(f"{'═'*60}")
    print("  The arm's motors (1–4) will be DISABLED so you can move")
    print("  the arm freely by hand.  Motor 5 (claw) stays locked.")
    print()
    print("  ⚠️  WARNING: The arm will FALL under gravity when torque")
    print("  is disabled!  SUPPORT THE ARM with your hand before")
    print("  pressing Enter to disable torque.")
    print()
    print("  For each ball:")
    print("    1. Support the arm, then press Enter to go limp.")
    print("    2. Guide the end-effector so it touches the ball.")
    print("    3. Press Enter to record the position.")
    print("    4. Torque re-engages automatically to hold position.")
    print("    5. Enter to accept, or 'r' to retry the ball.")
    print(f"{'═'*60}")

    try:
        for i in range(n):
            px, py = pixel_points[i]

            while True:  # retry loop for this ball
                print(f"\n  👉  Ball {i+1}/{n} — Move arm to ball at pixel ({px:.0f}, {py:.0f})")
                print("       ⚠️  SUPPORT the arm, then press Enter to disable torque...")
                input()

                # ── Disable torque on motors 1–4 (keep m5 claw torqued) ──
                try:
                    for motor_id in range(1, 5):
                        cmd = json.dumps({"cmd": "set_torque", "id": motor_id, "enable": False})
                        ser.write((cmd + "\n").encode())
                        resp = ser.readline().decode(errors="replace").strip()
                        if "ERR" in resp:
                            print(f"  ⚠ WARNING: Motor {motor_id} torque disable failed: {resp}")
                    print("       🔓 Motors 1–4 are now LIMP — guide the arm to the ball.")
                except Exception as e:
                    print(f"       ❌ Error disabling torque: {e}")
                    # Try to re-enable torque as safety fallback
                    try:
                        cmd = json.dumps({"cmd": "enable_torque"})
                        ser.write((cmd + "\n").encode())
                        ser.readline()
                    except Exception:
                        pass
                    print("       Retrying this ball...")
                    continue

                print("       Press Enter when the end-effector is touching the ball...")
                input()

                # ── Read positions and immediately re-enable torque ──────
                positions = None
                try:
                    # Read current motor positions
                    cmd = json.dumps({"cmd": "read_pos"})
                    ser.write((cmd + "\n").encode())
                    resp = ser.readline().decode(errors="replace").strip()

                    try:
                        positions = json.loads(resp)
                    except json.JSONDecodeError:
                        print(f"       ❌ Invalid response from read_pos: {resp}")
                except Exception as e:
                    print(f"       ❌ Error reading positions: {e}")
                finally:
                    # ALWAYS re-enable torque so the arm holds position
                    try:
                        # FIX: Write current positions as Goal Positions BEFORE
                        # re-enabling torque.  Without this, motors snap back to
                        # the stale Goal Position (e.g. 800 from SCAN_POSE for M4)
                        # which can be hundreds of steps away from where the user
                        # physically placed the arm — causing an overload error
                        # (red blinking LED) on the XL430 wrist motor.
                        if positions is not None:
                            print(f"       [DEBUG] Setting goal positions to current: {positions}")
                            send_raw_command(ser, positions)
                        else:
                            # Positions weren't read — read them now as a fallback
                            try:
                                cmd = json.dumps({"cmd": "read_pos"})
                                ser.write((cmd + "\n").encode())
                                fallback_resp = ser.readline().decode(errors="replace").strip()
                                fallback_pos = json.loads(fallback_resp)
                                print(f"       [DEBUG] Fallback goal positions: {fallback_pos}")
                                send_raw_command(ser, fallback_pos)
                            except Exception:
                                print("       [DEBUG] Could not read fallback positions — torque-on may snap!")

                        cmd = json.dumps({"cmd": "enable_torque"})
                        ser.write((cmd + "\n").encode())
                        ser.readline()
                        print("       🔒 Torque re-enabled — arm is holding position.")
                    except Exception as e2:
                        print(f"       ⚠️  Failed to re-enable torque: {e2}")
                        print("       ⚠️  MANUALLY support the arm!")

                if positions is None:
                    print("       ⚠️  Could not read positions. Retrying this ball...")
                    continue

                # Validate that we got all required motor keys
                required_keys = {"m1", "m2", "m3", "m4"}
                if not required_keys.issubset(positions.keys()):
                    missing = required_keys - set(positions.keys())
                    print(f"       ❌ Missing motor data: {missing}. Response: {positions}")
                    print("       Retrying this ball...")
                    continue

                # ── Forward kinematics ───────────────────────────────────
                try:
                    fk = arm.forward_kinematics(positions)
                    fk_x, fk_y, fk_z = fk["x"], fk["y"], fk["z"]

                    # Calculate relative m4_offset (delta from IK).
                    # The calibration system expects offsets relative to the IK baseline.
                    try:
                        ik_sol = arm.solve(fk_x, fk_y, fk_z, skip_sag=True)
                        fk_m4_offset = int(positions["m4"] - ik_sol["m4"])
                    except Exception:
                        fk_m4_offset = 0
                except Exception as e:
                    print(f"       ❌ Forward kinematics error: {e}")
                    print("       Retrying this ball...")
                    continue

                print(f"       📐 Motor positions: m1={positions['m1']}, m2={positions['m2']}, "
                      f"m3={positions['m3']}, m4={positions['m4']}")
                print(f"       📍 Computed:  X={fk_x:.2f} cm,  Y={fk_y:.2f} cm,  "
                      f"Z={fk_z:.2f} cm,  m4_offset={fk_m4_offset:+d}")

                # ── Confirm or retry ─────────────────────────────────────
                confirm = input("       Accept? (Enter=yes, r=retry): ").strip().lower()
                if confirm == 'r':
                    print("       🔄 Retrying this ball...")
                    continue

                # ── Store point (same format as _touch_phase) ────────────
                cm_points.append((fk_x, fk_y, fk_z, fk_m4_offset))
                print(f"       ✅ Saved Ball {i+1}/{n} at ({fk_x:.2f}, {fk_y:.2f}, "
                      f"z={fk_z:.2f}) cm, m4_offset={fk_m4_offset:+d}")
                break  # move to next ball
    except KeyboardInterrupt:
        print("\n⚠ Interrupted by user.")
    finally:
        # Safety: always try to re-enable torque on exit
        print("  Re-enabling torque on all motors...")
        try:
            # Read current positions and set as goals to prevent snap-back
            cmd = json.dumps({"cmd": "read_pos"})
            ser.write((cmd + "\n").encode())
            exit_resp = ser.readline().decode(errors="replace").strip()
            try:
                exit_pos = json.loads(exit_resp)
                send_raw_command(ser, exit_pos)
            except (json.JSONDecodeError, Exception):
                pass  # Best-effort; if it fails we still enable torque

            cmd = json.dumps({"cmd": "enable_torque"})
            ser.write((cmd + "\n").encode())
            ser.readline()
            print("  ✔ Torque re-enabled.")
        except Exception:
            print("  ⚠ WARNING: Could not re-enable torque! Manually power-cycle the arm.")

    # ── Summary ──────────────────────────────────────────────────────
    print(f"\n{'═'*60}")
    print(f"  📋  LIMP-MODE TOUCH SUMMARY — {len(cm_points)} points recorded")
    print(f"{'═'*60}")
    for idx, (cx, cy, cz, m4o) in enumerate(cm_points):
        px, py = pixel_points[idx]
        print(f"    #{idx+1}: pixel ({px:.0f}, {py:.0f})  →  "
              f"({cx:.2f}, {cy:.2f}, z={cz:.2f}) cm, m4_offset={m4o:+d}")
    print(f"{'═'*60}")

    return cm_points


# ══════════════════════════════════════════════════════════════════════
#  Phase 3 — Compute homography + reprojection error
# ══════════════════════════════════════════════════════════════════════

def _compute_homography(pixel_points: List[Tuple[float, float]],
                        cm_points: List[Tuple[float, float, float, int]]):
    """Compute the homography and print a reprojection error report.

    Uses ``cv2.getPerspectiveTransform`` for exactly 4 points, or
    ``cv2.findHomography(RANSAC)`` for 5+ points.

    Only the (x, y) components of *cm_points* are used for the homography;
    the Z and m4_offset components are recorded separately for height and
    wrist calibration.
    """
    # Extract only (x, y) for homography — Z and m4_offset are used separately
    cm_xy = [(x, y) for x, y, _z, _m4 in cm_points]
    px_array = np.float32(pixel_points)
    cm_array = np.float32(cm_xy)
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
        actual = np.float64([cm_xy[i][0], cm_xy[i][1]])
        err = float(np.linalg.norm(projected - actual))
        errors.append(err)

        status = "✅" if inlier_mask[i] else "❌ outlier"
        print(f"    Point #{i+1}: pixel ({pixel_points[i][0]:.0f}, {pixel_points[i][1]:.0f}) "
              f"→ expected ({cm_xy[i][0]:.1f}, {cm_xy[i][1]:.1f}) cm "
              f"→ got ({projected[0]:.1f}, {projected[1]:.1f}) cm "
              f"→ error {err:.2f} cm  {status}")

    mean_err = float(np.mean(errors))
    max_err = float(np.max(errors))
    if n == 4:
        print(f"\n    Mean reprojection error: {mean_err:.3f} cm  (always 0 for exact 4-point fit)")
        print(f"    Max  reprojection error: {max_err:.3f} cm  (always 0 for exact 4-point fit)")
    else:
        print(f"\n    Mean reprojection error: {mean_err:.3f} cm")
        print(f"    Max  reprojection error: {max_err:.3f} cm")

    if n == 4:
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
    return homography, inlier_mask


# ══════════════════════════════════════════════════════════════════════
#  Save calibration
# ══════════════════════════════════════════════════════════════════════

def _save_calibration(pixel_points, cm_points, homography, inlier_mask=None):
    # If no mask provided, treat all points as inliers (backward compatibility)
    if inlier_mask is None:
        inlier_mask = np.ones(len(pixel_points), dtype=bool)

    n_outliers = int((~inlier_mask).sum())
    if n_outliers > 0:
        print(f"\n  ⚠️  Excluded {n_outliers} outlier point(s) from calibration data")

    # Filter to inliers only for all auxiliary data
    filtered_px = [pt for pt, keep in zip(pixel_points, inlier_mask) if keep]
    filtered_cm = [pt for pt, keep in zip(cm_points, inlier_mask) if keep]

    # Build height_calibration and wrist_calibration from inlier (x, y, z, m4_offset) tuples
    height_calibration = []
    wrist_calibration = []
    for x, y, z, m4_off in filtered_cm:
        distance = math.sqrt(x ** 2 + y ** 2)
        height_calibration.append({
            "distance": round(distance, 2),
            "z": round(z, 2),
        })
        wrist_calibration.append({
            "distance": round(distance, 2),
            "m4_offset": int(m4_off),
        })

    data = {
        "calibrated_at_scan_pose": {k: int(v) for k, v in SCAN_POSE.items()},
        "tolerance": int(SCAN_POSE_TOLERANCE),
        "workspace_px": [[round(x, 1), round(y, 1)] for x, y in filtered_px],
        # workspace_cm stays (x, y) only for backward compatibility with homography
        "workspace_cm": [[round(x, 2), round(y, 2)] for x, y, _z, _m4 in filtered_cm],
        "homography": homography.tolist(),
        "height_calibration": height_calibration,
        "wrist_calibration": wrist_calibration,
        "calibration_date": date.today().isoformat(),
        "n_calibration_points": len(filtered_px),
    }
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  💾  Calibration saved to {CALIBRATION_FILE}")
    print(f"       Height calibration: {len(height_calibration)} inlier points saved.")
    print(f"       Wrist calibration:  {len(wrist_calibration)} inlier points saved.")


# ══════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════

def main():
    ser = None
    try:
        print("╔══════════════════════════════════════════════════════════════╗")
        print("║     TOUCH CALIBRATION v2 — Auto-Detect + Multi-Point        ║")
        print("╠══════════════════════════════════════════════════════════════╣")
        print("║  Place 4–20 balls on the workspace, spread as wide as       ║")
        print("║  possible. More balls = more accurate calibration.          ║")
        print("║                                                              ║")
        print("║  Step 1: Arm moves to SCAN_POSE.                            ║")
        print("║  Step 2: Balls are auto-detected (or click manually).       ║")
        print("║  Step 3: Drive the arm to touch each ball (WASD + ENTER).   ║")
        print("║  Step 4: Homography computed with error report.             ║")
        print("╚══════════════════════════════════════════════════════════════╝\n")

        # Ask how many balls
        while True:
            n_input = input(f"How many balls are on the desk? [{MIN_POINTS}–{MAX_POINTS}] (default 8): ").strip()
            if not n_input:
                n_balls = 8
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
            ser = _open_serial()
            _move_to_scan_pose(ser)
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
        print(f"\n{'─'*60}")
        print("  Choose touch method:")
        print("    [1] WASD keyboard control (motors active)")
        print("    [2] Limp mode + WASD fine-tuning (two phases)")
        print(f"{'─'*60}")
        while True:
            choice = input("  Enter choice [1/2] (default 1): ").strip()
            if choice in ("", "1"):
                cm_points = _touch_phase(ser, arm, pixel_points)
                break
            elif choice == "2":
                print("\n  [Phase 2a] Coarse Touch (Limp Mode)")
                coarse_cm_points = _limp_touch_phase(ser, arm, pixel_points)
                
                print("\n  [Phase 2b] Fine-Tune (WASD Mode)")
                print("  Returning to SCAN_POSE before fine-tuning...")
                _move_to_scan_pose(ser)
                
                cm_points = _touch_phase(ser, arm, pixel_points, coarse_cm_points=coarse_cm_points)
                break
            else:
                print("  ⚠️  Please enter 1 or 2.")

        # ── Phase 3: Compute Homography ───────────────────────────────────
        homography, inlier_mask = _compute_homography(pixel_points, cm_points)

        # ── Save ──────────────────────────────────────────────────────────
        _save_calibration(pixel_points, cm_points, homography, inlier_mask)

        # Return to scan pose when done
        _move_to_scan_pose(ser)
        print("\n  Done. You can now run main.py! ✅\n")

    finally:
        if ser is not None:
            try:
                ser.close()
                print("\n🔌 Serial connection closed.")
            except Exception:
                pass


if __name__ == "__main__":
    main()
