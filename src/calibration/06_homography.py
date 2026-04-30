"""
06_homography.py
========================
Interactive tool to calibrate the 4-point homography that maps camera
pixels → shoulder-relative centimetres.

**Wrist-mounted camera workflow** — the arm moves to ``SCAN_POSE``
before the camera opens so the wrist-mounted OAK-D sees the full
workspace from the correct vantage point.

Usage
-----
    python -m src.calibration.06_homography

Workflow
--------
  0. The script connects to the OpenRB-150 and moves the arm to SCAN_POSE.
  1. The camera opens and shows a live feed.
  2. Click the 4 workspace corners IN ORDER:
       TL (top-left) → TR (top-right) → BR (bottom-right) → BL (bottom-left)
  3. The camera window closes.
  4. In the terminal, type the physical (x, y) distance in cm from the
     SHOULDER JOINT (motor 2 pivot) for each corner.
  5. After 4 points the tool:
       a. Computes the homography.
       b. Prints the exact WORKSPACE_PX / WORKSPACE_CM Python code to
          paste into vision_bridge.py.
       c. Saves calibration (including SCAN_POSE metadata) to
          homography_calibration.json.
       d. Reopens the camera for live verification with ball detection.

Press 'r' during the click phase to reset all points and start over.
Press 'q' or Ctrl+C at any time to quit.

Author: Bachelor Project 2026 – Autonomia
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import time
from datetime import date
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


from vision.camera import OAKCamera
from vision.detector import SimpleBallDetector, BallColor
from config import vision as vcfg
from config.arm import SCAN_POSE, SCAN_POSE_TOLERANCE

# ── Constants ─────────────────────────────────────────────────────────
CORNER_NAMES = ["TL (top-left)", "TR (top-right)", "BR (bottom-right)", "BL (bottom-left)"]
CORNER_COLORS = [(0, 255, 0), (255, 255, 0), (0, 0, 255), (255, 0, 255)]
WINDOW_NAME = "Homography Calibration"
CALIBRATION_FILE = Path(__file__).resolve().parent / "homography_calibration.json"

# ── Serial settings (same as 02_joints.py / 02b_claw.py) ─────────────
SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD = 115200

_ser = None  # lazily initialised on first call

# ── Global state for mouse callback ──────────────────────────────────
_clicked_points: List[Tuple[int, int]] = []
_current_mouse: Tuple[int, int] = (0, 0)


# ── Serial helpers (pattern from 02_joints.py) ───────────────────────

def _get_serial():
    """Return the shared serial connection, opening it on first use."""
    global _ser
    if _ser is None:
        import serial
        print(f"[SERIAL] Opening {SERIAL_PORT} @ {SERIAL_BAUD} …")
        _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=2)
        time.sleep(3)  # wait for OpenRB-150 to boot

        # Drain any boot messages
        boot_msg = ""
        while _ser.in_waiting:
            boot_msg += _ser.readline().decode(errors="replace").strip() + " "
        if not boot_msg.strip():
            boot_msg = _ser.readline().decode(errors="replace").strip()
        print(f"[SERIAL] OpenRB says: {boot_msg.strip()}")

        # Re-enable torque
        cmd = json.dumps({"cmd": "enable_torque"})
        _ser.write((cmd + "\n").encode())
        _ser.readline()

        # Set a conservative motion profile so large jumps are slow
        cmd = json.dumps({"cmd": "set_profile", "vel": 40, "acc": 10})
        _ser.write((cmd + "\n").encode())
        _ser.readline()
        print("[SERIAL] Ready (profile: vel=40, acc=10)")
    return _ser


def send_command(positions: dict):
    """Send a dict of motor positions (e.g. {"m1": 2048, …}) to the OpenRB.

    The firmware expects a JSON object with keys m1–m5 containing
    Dynamixel step values (0–4095).  Returns the firmware response string.
    """
    ser = _get_serial()
    cmd_json = json.dumps(positions)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    if resp != "OK":
        print(f"  ⚠️  Unexpected response: {resp}")
    return resp


def _move_to_scan_pose():
    """Connect to the OpenRB-150 and command the arm to SCAN_POSE.

    Waits 2 seconds after sending the command for the motion to settle.
    """
    print("[ARM] Moving arm to SCAN_POSE for calibration...")
    pose_str = ", ".join(f"{k}={v}" for k, v in SCAN_POSE.items())
    print(f"[ARM] Target: {pose_str}")
    send_command(SCAN_POSE)
    time.sleep(2)  # wait for motion to settle
    print("[ARM] ✅ Arm is at SCAN_POSE.\n")


# ── Mouse / drawing helpers ──────────────────────────────────────────

def _mouse_callback(event, x, y, flags, param):
    """Handle mouse events on the calibration window."""
    global _current_mouse
    _current_mouse = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(_clicked_points) < 4:
            _clicked_points.append((x, y))
            print(f"  ✅  Clicked corner {len(_clicked_points)}/4: "
                  f"{CORNER_NAMES[len(_clicked_points)-1]} at pixel ({x}, {y})")


def _draw_overlay(frame: np.ndarray, points: List[Tuple[int, int]],
                  mouse: Tuple[int, int]) -> np.ndarray:
    """Draw calibration overlay on the frame."""
    overlay = frame.copy()
    h, w = overlay.shape[:2]

    # Draw instruction banner
    n_done = len(points)
    if n_done < 4:
        label = f"Click corner {n_done+1}/4: {CORNER_NAMES[n_done]}"
        cv2.rectangle(overlay, (0, 0), (w, 36), (0, 0, 0), -1)
        cv2.putText(overlay, label, (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    else:
        cv2.rectangle(overlay, (0, 0), (w, 36), (0, 80, 0), -1)
        cv2.putText(overlay, "All 4 corners clicked! Window closing...", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    # Draw clicked points with labels
    for i, (px, py) in enumerate(points):
        color = CORNER_COLORS[i]
        cv2.circle(overlay, (px, py), 8, (0, 0, 0), -1)
        cv2.circle(overlay, (px, py), 6, color, -1)
        cv2.putText(overlay, CORNER_NAMES[i], (px + 12, py + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    # Draw lines between clicked points
    for i in range(len(points)):
        j = (i + 1) % len(points)
        if j < len(points):
            cv2.line(overlay, points[i], points[j], (100, 100, 100), 1)
    if len(points) == 4:
        cv2.line(overlay, points[3], points[0], (100, 100, 100), 1)

    # Draw crosshair at mouse position
    mx, my = mouse
    cv2.line(overlay, (mx - 15, my), (mx + 15, my), (255, 255, 255), 1)
    cv2.line(overlay, (mx, my - 15), (mx, my + 15), (255, 255, 255), 1)
    cv2.putText(overlay, f"({mx}, {my})", (mx + 10, my - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

    return overlay


def _ask_physical_coordinates(corner_name: str, px: int, py: int) -> Tuple[float, float]:
    """Prompt the user for the physical (x, y) of a corner."""
    print(f"\n  📏  Corner: {corner_name}  (pixel: {px}, {py})")
    print(f"       Measure from the SHOULDER JOINT (motor 2 pivot):")
    print(f"         x = forward distance (cm)")
    print(f"         y = left(+) / right(−) distance (cm)")
    while True:
        try:
            x_str = input(f"       x (cm): ").strip()
            y_str = input(f"       y (cm): ").strip()
            x = float(x_str)
            y = float(y_str)
            return x, y
        except ValueError:
            print("       ❌ Invalid number, try again.")


def _print_code_snippet(pixel_points: List[Tuple[int, int]],
                        cm_points: List[Tuple[float, float]]):
    """Print the Python code to paste into vision_bridge.py."""
    print(f"\n{'═'*60}")
    print("  📋  Calibration values:")
    print(f"{'═'*60}\n")
    print("WORKSPACE_PX = np.float32([")
    for i, (px, py) in enumerate(pixel_points):
        name = CORNER_NAMES[i]
        print(f"    [{px:4d}, {py:4d}],      # {name}")
    print("])\n")
    print("WORKSPACE_CM = np.float32([")
    for i, (x, y) in enumerate(cm_points):
        name = CORNER_NAMES[i]
        print(f"    [{x:5.1f}, {y:5.1f}],   # {name}")
    print("])")
    print(f"\n{'═'*60}\n")


def _save_calibration(pixel_points: List[Tuple[int, int]],
                      cm_points: List[Tuple[float, float]],
                      homography: np.ndarray) -> None:
    """Save calibration data to a JSON file for automatic loading.

    The file is written to ``CALIBRATION_FILE`` (``homography_calibration.json``
    next to this script).  ``vision_bridge.py`` loads it at import time so
    no manual editing of source files is needed.

    The expanded schema includes ``calibrated_at_scan_pose`` and
    ``tolerance`` so downstream code can verify the arm is at the correct
    pose before trusting the homography.
    """
    data = {
        "calibrated_at_scan_pose": {k: int(v) for k, v in SCAN_POSE.items()},
        "tolerance": int(SCAN_POSE_TOLERANCE),
        "workspace_px": [list(map(int, pt)) for pt in pixel_points],
        "workspace_cm": [list(map(float, pt)) for pt in cm_points],
        "homography": homography.tolist(),
        "calibration_date": date.today().isoformat(),
    }
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\n  💾  Calibration saved to {CALIBRATION_FILE}")
    print("       vision_bridge.py will load it automatically. No manual editing needed.")


def _run_verification(cam: OAKCamera, homography: np.ndarray):
    """Run a live verification loop showing detected ball coordinates."""
    print("\n  🔍  VERIFICATION MODE")
    print("       Detected ball coordinates are now shoulder-relative.")
    print("       Place a ball at a known position and check the readout.")
    print("       Press 'q' to quit.\n")

    focal_px = cam.get_focal_length_px(hfov_deg=vcfg.CAMERA_HFOV_DEG)
    detector = SimpleBallDetector(
        min_radius=vcfg.BALL_MIN_RADIUS,
        max_radius=vcfg.BALL_MAX_RADIUS,
        confidence_threshold=vcfg.BALL_CONFIDENCE_THRESHOLD,
        enable_adaptive_lighting=True,
        max_balls_per_color=4,
        focal_length_px=focal_px,
    )

    scan_num = 0
    while True:
        ret, frame = cam.read()
        if not ret or frame is None:
            continue

        scan_num += 1
        balls, _ = detector.detect_balls(frame)

        overlay = frame.copy()
        for ball in balls:
            colour_name = ball.color.value
            if colour_name == "unknown":
                continue

            cx, cy = ball.center
            radius = int(ball.radius)
            bgr = (0, 0, 255) if colour_name == "red" else (255, 130, 0)

            # Transform pixel → cm using the new homography
            point = np.float32([[[float(cx), float(cy)]]])
            transformed = cv2.perspectiveTransform(point, homography)
            x_cm = round(float(transformed[0, 0, 0]), 1)
            y_cm = round(float(transformed[0, 0, 1]), 1)
            reach = round((x_cm**2 + y_cm**2)**0.5, 1)

            # Draw on frame
            cv2.circle(overlay, (cx, cy), radius, bgr, 2)
            cv2.circle(overlay, (cx, cy), 3, (255, 255, 255), -1)
            label = f"{colour_name.upper()} ({x_cm}, {y_cm}) cm  r={reach}"
            cv2.putText(overlay, label, (cx - radius, cy - radius - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1, cv2.LINE_AA)

            if scan_num % 10 == 1:
                print(f"  [{scan_num:3d}] {colour_name.upper():5s}  "
                      f"x={x_cm:6.1f} cm   y={y_cm:6.1f} cm   "
                      f"reach={reach:5.1f} cm")

        cv2.imshow(WINDOW_NAME, overlay)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break


def main():
    global _clicked_points

    print("╔══════════════════════════════════════════════════════════════╗")
    print("║        HOMOGRAPHY CALIBRATION TOOL  (wrist-mounted cam)    ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  This tool calibrates the pixel → cm mapping.              ║")
    print("║                                                            ║")
    print("║  Step 0: Arm moves to SCAN_POSE (wrist camera overhead)    ║")
    print("║  Step 1: Click 4 corners on the camera feed (TL→TR→BR→BL) ║")
    print("║  Step 2: Type the physical (x, y) from shoulder joint      ║")
    print("║  Step 3: Verify with a ball at a known position            ║")
    print("║                                                            ║")
    print("║  Keys: 'r' = reset | 'q' = quit                           ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    # ── Step 0: Connect to OpenRB-150 and move arm to SCAN_POSE ───────
    print("[INIT] Connecting to OpenRB-150 and moving arm to SCAN_POSE...")
    try:
        _move_to_scan_pose()
    except Exception as e:
        print(f"❌ Could not connect to OpenRB-150 or move arm: {e}")
        print("   Check serial connection and try again.")
        return

    # ── Step 1: Open camera ───────────────────────────────────────────
    print("[INIT] Opening OAK-D camera...")
    cam = OAKCamera(resolution=vcfg.CAMERA_RESOLUTION)
    if not cam.open():
        print("❌ Could not open camera. Check OAK-D connection.")
        return

    focal_px = cam.get_focal_length_px(hfov_deg=vcfg.CAMERA_HFOV_DEG)
    print(f"[INIT] ✅ Camera ready ({vcfg.CAMERA_RESOLUTION[0]}×"
          f"{vcfg.CAMERA_RESOLUTION[1]}, f={focal_px:.1f}px)\n")

    # Set up window
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, _mouse_callback)

    # ── Phase 1: Click 4 corners ──────────────────────────────────────
    print("  🖱️  Click the 4 workspace corners in the camera feed:")
    for i, name in enumerate(CORNER_NAMES):
        print(f"       {i+1}. {name}")
    print()

    try:
        while len(_clicked_points) < 4:
            ret, frame = cam.read()
            if not ret or frame is None:
                continue

            overlay = _draw_overlay(frame, _clicked_points, _current_mouse)
            cv2.imshow(WINDOW_NAME, overlay)

            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                print("\n  ⛔ Quit requested.")
                cam.release()
                cv2.destroyAllWindows()
                return
            if key == ord('r'):
                _clicked_points.clear()
                print("\n  🔄 Reset — click corners again.\n")

        # Show the final frame with all 4 points for 1 second
        for _ in range(30):
            ret, frame = cam.read()
            if ret and frame is not None:
                overlay = _draw_overlay(frame, _clicked_points, _current_mouse)
                cv2.imshow(WINDOW_NAME, overlay)
            cv2.waitKey(33)

    except KeyboardInterrupt:
        print("\n  ⛔ Interrupted.")
        cam.release()
        cv2.destroyAllWindows()
        return

    pixel_points = list(_clicked_points)

    # ── CLOSE the camera window before asking for terminal input ──────
    # On macOS, an OpenCV window that isn't being pumped with waitKey()
    # causes the whole process to freeze/be killed by the OS.
    cam.release()
    cv2.destroyAllWindows()
    # Give macOS time to tear down the window
    time.sleep(0.5)

    # ── Phase 2: Ask physical coordinates (terminal only) ─────────────
    print(f"\n{'─'*60}")
    print("  📏  Now enter the physical coordinates for each corner.")
    # NOTE: Even though the camera is now wrist-mounted, the coordinate
    # frame is still centred on the SHOULDER JOINT (motor 2 pivot, on
    # the desk directly below the shoulder).  Measure WORKSPACE_CM
    # corners from that origin — the wrist camera position does not
    # change the reference frame.
    print("  ⚠️  IMPORTANT: Measure from the SHOULDER JOINT (motor 2 pivot)")
    print("       on the desk directly below the shoulder — NOT from the camera.")
    print("       The wrist-mounted camera does NOT change the coordinate frame.")
    print(f"{'─'*60}")

    cm_points: List[Tuple[float, float]] = []
    for i, name in enumerate(CORNER_NAMES):
        px, py = pixel_points[i]
        x, y = _ask_physical_coordinates(name, px, py)
        cm_points.append((x, y))
        print(f"       → ({x:.1f}, {y:.1f}) cm from shoulder ✅")

    # ── Compute homography ────────────────────────────────────────────
    px_array = np.float32(pixel_points)
    cm_array = np.float32(cm_points)
    homography = cv2.getPerspectiveTransform(px_array, cm_array)

    print(f"\n{'═'*60}")
    print("  ✅  Homography computed!")
    print(f"{'═'*60}")

    # ── Print code snippet ────────────────────────────────────────────
    _print_code_snippet(pixel_points, cm_points)

    # ── Auto-save calibration to JSON (expanded schema) ───────────────
    _save_calibration(pixel_points, cm_points, homography)

    # ── Verification test with the clicked corners ────────────────────
    print("  🧪  Verification — checking corner mappings:")
    for i, (px, py) in enumerate(pixel_points):
        point = np.float32([[[float(px), float(py)]]])
        transformed = cv2.perspectiveTransform(point, homography)
        mapped_x = float(transformed[0, 0, 0])
        mapped_y = float(transformed[0, 0, 1])
        expected_x, expected_y = cm_points[i]
        err = ((mapped_x - expected_x)**2 + (mapped_y - expected_y)**2)**0.5
        status = "✅" if err < 0.5 else "⚠️"
        print(f"    {status}  {CORNER_NAMES[i]}: pixel ({px}, {py}) → "
              f"({mapped_x:.1f}, {mapped_y:.1f}) cm  "
              f"[expected ({expected_x:.1f}, {expected_y:.1f}), err={err:.2f}]")

    # ── Phase 3: Live verification with ball detection ────────────────
    print(f"\n{'─'*60}")
    ans = input("  Open camera for live verification? (y/n): ").strip().lower()
    if ans != 'y':
        print("\n  Done. Calibration saved — no manual editing needed. ✅\n")
        return

    print("\n[INIT] Reopening camera for verification...")
    # Re-send SCAN_POSE in case the arm drifted during terminal input
    print("[ARM] Re-sending SCAN_POSE to ensure arm is in position...")
    send_command(SCAN_POSE)
    time.sleep(1)

    cam2 = OAKCamera(resolution=vcfg.CAMERA_RESOLUTION)
    if not cam2.open():
        print("❌ Could not reopen camera.")
        print("\n  Done. Calibration saved — no manual editing needed. ✅\n")
        return

    _run_verification(cam2, homography)

    cam2.release()
    cv2.destroyAllWindows()
    print("\n  Done. Calibration saved — no manual editing needed. ✅\n")


if __name__ == "__main__":
    main()
