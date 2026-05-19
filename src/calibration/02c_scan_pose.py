"""
02c_scan_pose.py
================
Interactive SCAN_POSE tuning script for the wrist-mounted OAK-D S2.

Connects to the OpenRB-150, moves the arm to the current SCAN_POSE from
``src/config/arm.py``, then opens the OAK-D camera and shows a live RGB
feed.  Use the keyboard to nudge individual motors while watching the view
change in real time.  Press ``ENTER`` to write the tuned values back
to ``src/config/arm.py`` and exit.

Usage
-----
    python -m src.calibration.02c_scan_pose

Key controls (also overlaid on the live frame)
----------------------------------------------
    W / s  → m2 shoulder  +/- step  (raise / lower arm)
    E / D  → m3 elbow     +/- step  (fold / unfold forearm)
    R / F  → m4 wrist     +/- step  (tilt camera up / down)
    T / G  → m1 base      +/- step  (rotate left / right)
    Y / H  → m5 claw      +/- step  (open / close claw)
    [ / ]  → step size    ×2 / ÷2   (default 50, range 10–400)
    SPACE  → read & print current firmware positions
    ENTER  → SAVE to src/config/arm.py and exit
    Q      → quit without saving

Author: Bachelor Project 2026 – Autonomia
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import re
import time
from pathlib import Path

import cv2
import numpy as np

from vision.camera import OAKCamera
from config.arm import SCAN_POSE as _INITIAL_SCAN_POSE

# ── Constants ─────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD = 115200
WINDOW_NAME  = "SCAN_POSE Tuner"

# Path to arm config, resolved relative to this file so it works regardless
# of the working directory.
ARM_CONFIG_FILE = Path(__file__).resolve().parent.parent / "config" / "arm.py"

STEP_DEFAULT = 50
STEP_MIN     = 10
STEP_MAX     = 400
MOTOR_MIN    = 100   # avoids hitting hard mechanical limit at 0
MOTOR_MAX    = 3995  # avoids hitting hard mechanical limit at 4095

# ── Shared serial connection ──────────────────────────────────────────
_ser = None  # lazily initialised on first call


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

        # Re-enable torque (mirrors main.py smooth_startup)
        cmd = json.dumps({"cmd": "enable_torque"})
        _ser.write((cmd + "\n").encode())
        _ser.readline()

        # Conservative motion profile — slow and safe for interactive tuning
        cmd = json.dumps({"cmd": "set_profile", "vel": 30, "acc": 10})
        _ser.write((cmd + "\n").encode())
        _ser.readline()
        print("[SERIAL] Ready (profile: vel=30, acc=10)")
    return _ser


def send_command(positions: dict) -> str:
    """Send a dict of motor positions (e.g. ``{"m1": 2048, …}``) to the OpenRB.

    The firmware expects a JSON object with keys m1–m5 containing Dynamixel
    step values (0–4095).  Returns the firmware response string.
    """
    ser = _get_serial()
    cmd_json = json.dumps(positions)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    if resp != "OK":
        print(f"  ⚠️  Unexpected response: {resp!r}")
    return resp


def read_positions() -> dict:
    """Ask the firmware for the current motor positions.

    Returns a dict like ``{"m1": 2048, "m2": …, "m5": …}`` or ``None``
    on failure.
    """
    ser = _get_serial()
    cmd = json.dumps({"cmd": "read_pos"})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    try:
        return json.loads(resp)
    except (json.JSONDecodeError, TypeError):
        print(f"  ⚠️  Could not parse position response: {resp!r}")
        return None


# ── Motor helpers ─────────────────────────────────────────────────────

def clamp(val: int) -> int:
    """Clamp ``val`` to the safe motor range [MOTOR_MIN, MOTOR_MAX]."""
    return max(MOTOR_MIN, min(MOTOR_MAX, int(val)))


# ── Overlay drawing ───────────────────────────────────────────────────

def draw_overlay(frame: np.ndarray, pose: dict, step: int) -> np.ndarray:
    """Return a copy of *frame* with motor-value and controls overlay."""
    out = frame.copy()

    lines = [
        f"m1(base): {pose['m1']}    m2(shoulder): {pose['m2']}",
        f"m3(elbow): {pose['m3']}   m4(wrist): {pose['m4']}    m5(claw): {pose['m5']}",
        f"step: {step}",
        "W/S:m2  E/D:m3  R/F:m4  T/G:m1  Y/H:m5  [/]:step  ENTER:save  Q:quit",
    ]

    font   = cv2.FONT_HERSHEY_SIMPLEX
    line_h = 22
    bg_h   = len(lines) * line_h + 8

    # Semi-transparent dark bar so text is always readable
    roi   = out[0:bg_h, :]
    black = np.zeros_like(roi)
    cv2.addWeighted(roi, 0.35, black, 0.65, 0, roi)
    out[0:bg_h, :] = roi

    y = 18
    for line in lines:
        # Shrink font slightly for the long controls line
        scale = 0.42 if len(line) > 55 else 0.50
        cv2.putText(out, line, (8, y),
                    font, scale, (0, 255, 100), 1, cv2.LINE_AA)
        y += line_h

    return out


# ── Config file update ────────────────────────────────────────────────

def save_scan_pose(pose: dict) -> None:
    """Overwrite SCAN_POSE values in ``src/config/arm.py``.

    Uses a regex to locate the ``SCAN_POSE = { … }`` block and replaces each
    motor value in-place, preserving all surrounding comments and whitespace.
    """
    text = ARM_CONFIG_FILE.read_text(encoding="utf-8")

    # Sentinel: set to True inside the callback so we can distinguish
    # "block not found" from "block found but values unchanged".
    _matched = [False]

    def _replace_block(m: re.Match) -> str:
        _matched[0] = True
        block = m.group(0)
        for key, val in pose.items():
            # Replace the integer after  "mN":  keeping any trailing comment
            block = re.sub(
                rf'("{key}":\s*)\d+',
                lambda mm, v=val: mm.group(1) + str(v),
                block,
            )
        return block

    new_text = re.sub(
        r'SCAN_POSE\s*=\s*\{[^}]*\}',
        _replace_block,
        text,
        flags=re.DOTALL,
    )

    if not _matched[0]:
        print("  ⚠️  No changes written — could not locate SCAN_POSE block.")
        return

    ARM_CONFIG_FILE.write_text(new_text, encoding="utf-8")
    print(f"\n  ✅  SCAN_POSE saved to {ARM_CONFIG_FILE}")
    print(f"      New values: {pose}")


# ── Main loop ─────────────────────────────────────────────────────────

def main() -> None:
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║          SCAN_POSE INTERACTIVE TUNING TOOL                 ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  W/s=m2  E/D=m3  R/F=m4  T/G=m1  Y/H=m5                  ║")
    print("║  [/]=step  SPACE=read_pos  ENTER=save  Q=quit              ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    # ── Connect to arm and move to current SCAN_POSE ───────────────────
    print("[INIT] Connecting to OpenRB-150 and moving to SCAN_POSE …")
    try:
        pose = {k: clamp(v) for k, v in _INITIAL_SCAN_POSE.items()}
        send_command(pose)
        time.sleep(2)  # wait for motion to settle
        print(f"[ARM] ✅ Arm at SCAN_POSE: {pose}\n")
    except Exception as exc:
        print(f"❌ Could not connect to arm: {exc}")
        print("   Check SERIAL_PORT and USB connection, then try again.")
        return

    # ── Open OAK-D camera ─────────────────────────────────────────────
    print("[INIT] Opening OAK-D S2 camera …")
    cam = OAKCamera(resolution=(1280, 720))
    if not cam.open():
        print("❌ Could not open OAK-D camera.  Check USB connection.")
        global _ser
        if _ser is not None:
            try:
                _ser.close()
            except Exception:
                pass
            _ser = None
        return
    print("[INIT] ✅ Camera ready.\n")
    print("[TUNER] Live feed active.  Nudge motors with keyboard.")
    print("        SPACE = read firmware positions")
    print("        ENTER = save & exit   Q = quit without saving\n")

    step = STEP_DEFAULT
    cv2.namedWindow(WINDOW_NAME)

    try:
        while True:
            ret, frame = cam.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            cv2.imshow(WINDOW_NAME, draw_overlay(frame, pose, step))

            # Non-blocking key read; 0xFF means no key pressed
            key = cv2.waitKey(1) & 0xFF
            if key == 0xFF:
                continue

            moved = False

            # ── Motor nudge keys ────────────────────────────────────────
            #   Both uppercase and lowercase trigger motor nudges.
            #   ENTER (13) = save; Q/q = quit.
            if key in (ord('w'), ord('W')):          # m2 shoulder +
                pose['m2'] = clamp(pose['m2'] + step)
                moved = True
            elif key == ord('s'):                     # m2 shoulder - (lowercase only)
                pose['m2'] = clamp(pose['m2'] - step)
                moved = True
            elif key in (ord('e'), ord('E')):         # m3 elbow +
                pose['m3'] = clamp(pose['m3'] + step)
                moved = True
            elif key in (ord('d'), ord('D')):         # m3 elbow -
                pose['m3'] = clamp(pose['m3'] - step)
                moved = True
            elif key in (ord('r'), ord('R')):         # m4 wrist +
                pose['m4'] = clamp(pose['m4'] + step)
                moved = True
            elif key in (ord('f'), ord('F')):         # m4 wrist -
                pose['m4'] = clamp(pose['m4'] - step)
                moved = True
            elif key in (ord('t'), ord('T')):         # m1 base +
                pose['m1'] = clamp(pose['m1'] + step)
                moved = True
            elif key in (ord('g'), ord('G')):         # m1 base -
                pose['m1'] = clamp(pose['m1'] - step)
                moved = True
            elif key in (ord('y'), ord('Y')):         # m5 claw +
                pose['m5'] = clamp(pose['m5'] + step)
                moved = True
            elif key in (ord('h'), ord('H')):         # m5 claw -
                pose['m5'] = clamp(pose['m5'] - step)
                moved = True

            # ── Step size ───────────────────────────────────────────────
            elif key == ord('['):
                step = min(STEP_MAX, step * 2)
                print(f"[STEP] Size → {step}")
            elif key == ord(']'):
                step = max(STEP_MIN, step // 2)
                print(f"[STEP] Size → {step}")

            # ── Read positions from firmware ────────────────────────────
            elif key == ord(' '):
                pos = read_positions()
                if pos:
                    print(f"[POS]  Firmware: {pos}")
                else:
                    print("[POS]  Could not read positions from firmware.")

            # ── Save (ENTER key = 13) ───────────────────────────────────
            elif key == 13:  # ENTER
                print(f"\n[SAVE] Writing SCAN_POSE → {pose}")
                save_scan_pose(pose)
                break

            # ── Quit without saving ─────────────────────────────────────
            elif key in (ord('q'), ord('Q')):
                print("\n[QUIT] Exiting without saving.")
                break

            # Send updated pose to the arm immediately after a nudge
            if moved:
                send_command(pose)
                print(f"[ARM]  → {pose}")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] Ctrl+C received — shutting down.")
    except Exception as exc:  # noqa: BLE001
        print(f"\n[ERROR] Unexpected error: {exc}")
    finally:
        cam.release()
        cv2.destroyAllWindows()
        if _ser is not None:
            try:
                _ser.close()
            except Exception:
                pass
        print("[DONE] Resources released.")


if __name__ == "__main__":
    main()
