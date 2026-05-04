"""
10_bin_calibration.py
======================
Interactive tool to calibrate sorting bin positions (RED_BIN, BLUE_BIN,
REJECT_BIN) by driving the robot arm with WASD keys or manually (limp mode).

Instead of relying on hardcoded bin coordinates in ``config/arm.py``, this
script lets the operator physically drive the end-effector to each bin and
record the exact (x, y, z, m4_offset) drop position.  Calibrated positions
are saved to ``bin_calibration.json`` and can be loaded at runtime.

The control interface (serial communication, IK movement, WASD control loop,
limp mode, and HUD rendering) follows the exact patterns established in
``09_touch_calibration.py``.

Usage
-----
    python -m src.calibration.10_bin_calibration
    python src/calibration/10_bin_calibration.py
    python src/calibration/10_bin_calibration.py --port /dev/ttyUSB0 --baud 115200

Author: Bachelor Project 2026 – Autonomia
"""
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import argparse
import tempfile
import time
from datetime import date
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np

from config.arm import (
    SCAN_POSE, CLEARANCE_HEIGHT, HOME_POSITION, BINS,
    M3_SCAN_CURRENT_LIMIT, M3_DEFAULT_CURRENT_LIMIT,
)
from ik.solver import ArmIK

# ── Constants ─────────────────────────────────────────────────────────
BIN_CALIBRATION_FILE = Path(__file__).resolve().parent / "bin_calibration.json"
SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD = 115200
WINDOW_NAME = "Bin Calibration"

# Ordered list of bins to calibrate
BIN_NAMES = ["RED_BIN", "BLUE_BIN", "REJECT_BIN"]


# ══════════════════════════════════════════════════════════════════════
#  Atomic JSON save
# ══════════════════════════════════════════════════════════════════════

def _save_json_atomic(path, data):
    """Write JSON atomically — write to temp file, then rename."""
    path_str = str(path)
    dir_name = os.path.dirname(path_str) or "."
    tmp_fd, tmp_path = tempfile.mkstemp(dir=dir_name, suffix=".tmp")
    try:
        with os.fdopen(tmp_fd, "w") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
        os.replace(tmp_path, path_str)
    except BaseException:
        os.unlink(tmp_path)
        raise


# ══════════════════════════════════════════════════════════════════════
#  Serial / Arm helpers
# ══════════════════════════════════════════════════════════════════════

def _open_serial(port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
    """Open a new serial connection to the arm and return it.

    This is called once in ``main()``; the returned object is passed as a
    parameter to every function that needs to talk to hardware.  Importing
    this module does **not** open a serial port.
    """
    import serial
    print(f"[SERIAL] Opening {port} @ {baud} …")
    ser = serial.Serial(port, baud, timeout=2)
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
    """Solve IK, apply an m4 offset, clamp, send command, and return (solution, response)."""
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
    print("[ARM] Moving arm to SCAN_POSE …")
    send_raw_command(ser, SCAN_POSE)
    time.sleep(2)
    if M3_SCAN_CURRENT_LIMIT > 0:
        print(f"[ARM] Applying M3 thermal limit ({M3_SCAN_CURRENT_LIMIT} mA)")
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 3, "value": M3_SCAN_CURRENT_LIMIT})
    print("[ARM] ✅ Arm is at SCAN_POSE.\n")


# ══════════════════════════════════════════════════════════════════════
#  WASD control loop for a single bin
# ══════════════════════════════════════════════════════════════════════

def _wasd_bin_phase(ser, arm: ArmIK, bin_name: str, bin_index: int,
                    total_bins: int,
                    start_x: float, start_y: float, start_z: float
                    ) -> Optional[Tuple[float, float, float, int]]:
    """Drive the arm with WASD keys to calibrate a single bin position.

    Returns (x, y, z, m4_offset) on ENTER, or None if the user quits.
    """
    if M3_DEFAULT_CURRENT_LIMIT > 0:
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 3, "value": M3_DEFAULT_CURRENT_LIMIT})

    target_x = start_x
    target_y = start_y
    target_z = start_z
    step = 1.0
    step_m4 = 10
    m4_offset = 0

    cv2.namedWindow(WINDOW_NAME)

    # Slow profile for initial move
    send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})

    # Move to clearance height above target first
    send_ik_command(ser, arm, target_x, target_y, CLEARANCE_HEIGHT)
    time.sleep(0.5)

    # Lower to target position
    send_ik_with_m4_offset(ser, arm, target_x, target_y, target_z, m4_offset)
    time.sleep(0.5)

    # Restore normal velocity for WASD manual driving
    send_raw_command(ser, {"cmd": "set_profile", "vel": 80, "acc": 20})

    while True:
        # ── Draw HUD ──────────────────────────────────────────────
        hud = np.zeros((450, 600, 3), dtype=np.uint8)

        cv2.putText(hud, f"Calibrating: {bin_name}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(hud, f"Bin {bin_index + 1}/{total_bins}", (450, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1)

        cv2.putText(hud, f"X = {target_x:.1f} cm  (forward/back)",
                    (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(hud, f"Y = {target_y:.1f} cm  (left/right)",
                    (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.putText(hud, f"Z = {target_z:.1f} cm  (up/down)",
                    (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        if target_z <= arm.Z_MIN:
            cv2.putText(hud, "Z at minimum! (IK clamp)",
                        (320, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(hud, f"m4 offset = {m4_offset:+d} steps  (wrist tilt)",
                    (20, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 1)
        cv2.putText(hud, f"Step Size = {step:.2f} cm",
                    (20, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)

        cv2.putText(hud, "W/S: X   A/D: Y   U/J: Z   I/K: Wrist",
                    (20, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(hud, "[/]: Step   ENTER: Save   L: Limp   Q: Quit",
                    (20, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(hud, f"Move arm to the {bin_name} drop position",
                    (20, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

        cv2.imshow(WINDOW_NAME, hud)
        key = cv2.waitKey(50) & 0xFF

        if key == 255:
            continue

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
            print(f"       ✅ Saved {bin_name} at ({target_x:.1f}, {target_y:.1f}, "
                  f"z={target_z:.1f}) cm, m4_offset={m4_offset:+d}")
            cv2.destroyAllWindows()
            return (target_x, target_y, target_z, m4_offset)
        elif key in (ord('l'), ord('L')):
            # Switch to limp mode for this bin
            cv2.destroyAllWindows()
            result = _limp_bin_phase(ser, arm, bin_name, bin_index, total_bins)
            if result is not None:
                return result
            # If limp mode was cancelled, re-enter WASD loop
            cv2.namedWindow(WINDOW_NAME)
            send_raw_command(ser, {"cmd": "set_profile", "vel": 80, "acc": 20})
            send_ik_with_m4_offset(ser, arm, target_x, target_y, target_z, m4_offset)
            time.sleep(0.3)
            continue
        elif key in (ord('q'), ord('Q')):
            print("\n  ⛔ Quit requested.")
            cv2.destroyAllWindows()
            return None

        if moved or wrist_moved:
            solution, resp = send_ik_with_m4_offset(ser, arm, target_x, target_y, target_z, m4_offset)
            if solution is None:
                target_x, target_y, target_z = old_x, old_y, old_z
                m4_offset = old_m4
                print("       ⚠️  Move reverted (unreachable).")


# ══════════════════════════════════════════════════════════════════════
#  Limp-mode calibration for a single bin
# ══════════════════════════════════════════════════════════════════════

def _limp_bin_phase(ser, arm: ArmIK, bin_name: str, bin_index: int,
                    total_bins: int
                    ) -> Optional[Tuple[float, float, float, int]]:
    """Record a bin position by manually guiding the limp arm.

    Disables torque on motors 1–4 so the user can physically move the arm
    by hand to the bin position.  Motor 5 (claw) stays torqued.

    Forward kinematics converts the read motor positions to (x, y, z,
    m4_offset) — the same format as the WASD phase.

    Returns (x, y, z, m4_offset) on success, or None if cancelled.
    """
    if M3_DEFAULT_CURRENT_LIMIT > 0:
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 3, "value": M3_DEFAULT_CURRENT_LIMIT})

    print(f"\n{'═'*60}")
    print(f"  🖐️  LIMP MODE — {bin_name}  (Bin {bin_index + 1}/{total_bins})")
    print(f"{'═'*60}")
    print("  The arm's motors (1–4) will be DISABLED so you can move")
    print("  the arm freely by hand.  Motor 5 (claw) stays locked.")
    print()
    print("  ⚠️  WARNING: The arm will FALL under gravity when torque")
    print("  is disabled!  SUPPORT THE ARM with your hand before")
    print("  pressing Enter to disable torque.")
    print()
    print("  Steps:")
    print(f"    1. Support the arm, then press Enter to go limp.")
    print(f"    2. Guide the end-effector to the {bin_name} position.")
    print(f"    3. Press Enter to record the position.")
    print(f"    4. Torque re-engages automatically.")
    print(f"    5. Enter to accept, or 'r' to retry, 'q' to cancel.")
    print(f"{'═'*60}")

    while True:  # retry loop
        print(f"\n  👉  {bin_name} — SUPPORT the arm, then press Enter to disable torque…")
        input()

        # ── Disable torque on motors 1–4 (keep m5 claw torqued) ──
        try:
            for motor_id in range(1, 5):
                cmd = json.dumps({"cmd": "set_torque", "id": motor_id, "enable": False})
                ser.write((cmd + "\n").encode())
                resp = ser.readline().decode(errors="replace").strip()
                if "ERR" in resp:
                    print(f"  ⚠ WARNING: Motor {motor_id} torque disable failed: {resp}")
            print("       🔓 Motors 1–4 are now LIMP — guide the arm to the bin.")
        except Exception as e:
            print(f"       ❌ Error disabling torque: {e}")
            try:
                cmd = json.dumps({"cmd": "enable_torque"})
                ser.write((cmd + "\n").encode())
                ser.readline()
            except Exception:
                pass
            print("       Retrying…")
            continue

        print(f"       Press Enter when the end-effector is at the {bin_name} position…")
        input()

        # ── Read positions and immediately re-enable torque ──────
        positions = None
        try:
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
                if positions is not None:
                    print(f"       [DEBUG] Setting goal positions to current: {positions}")
                    send_raw_command(ser, positions)
                else:
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
            print("       ⚠️  Could not read positions. Retrying…")
            continue

        # Validate motor keys
        required_keys = {"m1", "m2", "m3", "m4"}
        if not required_keys.issubset(positions.keys()):
            missing = required_keys - set(positions.keys())
            print(f"       ❌ Missing motor data: {missing}. Response: {positions}")
            print("       Retrying…")
            continue

        # ── Forward kinematics ───────────────────────────────────
        try:
            fk = arm.forward_kinematics(positions)
            fk_x, fk_y, fk_z = fk["x"], fk["y"], fk["z"]

            # Calculate relative m4_offset (delta from IK baseline)
            try:
                ik_sol = arm.solve(fk_x, fk_y, fk_z, skip_sag=True)
                fk_m4_offset = int(positions["m4"] - ik_sol["m4"])
            except Exception:
                fk_m4_offset = 0
        except Exception as e:
            print(f"       ❌ Forward kinematics error: {e}")
            print("       Retrying…")
            continue

        print(f"       📐 Motor positions: m1={positions['m1']}, m2={positions['m2']}, "
              f"m3={positions['m3']}, m4={positions['m4']}")
        print(f"       📍 Computed:  X={fk_x:.2f} cm,  Y={fk_y:.2f} cm,  "
              f"Z={fk_z:.2f} cm,  m4_offset={fk_m4_offset:+d}")

        # ── Confirm or retry ─────────────────────────────────────
        confirm = input("       Accept? (Enter=yes, r=retry, q=cancel): ").strip().lower()
        if confirm == 'r':
            print("       🔄 Retrying…")
            continue
        elif confirm == 'q':
            print("       ⛔ Limp mode cancelled.")
            return None

        print(f"       ✅ Saved {bin_name} at ({fk_x:.2f}, {fk_y:.2f}, "
              f"z={fk_z:.2f}) cm, m4_offset={fk_m4_offset:+d}")
        return (fk_x, fk_y, fk_z, fk_m4_offset)


# ══════════════════════════════════════════════════════════════════════
#  Save calibration
# ══════════════════════════════════════════════════════════════════════

def _save_calibration(calibrated_bins: dict):
    """Save all calibrated bin positions to bin_calibration.json."""
    data = {
        "bins": calibrated_bins,
        "calibration_date": date.today().isoformat(),
        "calibrated_with_scan_pose": {k: int(v) for k, v in SCAN_POSE.items()},
    }
    _save_json_atomic(BIN_CALIBRATION_FILE, data)
    print(f"\n  💾  Calibration saved to {BIN_CALIBRATION_FILE}")


# ══════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Calibrate sorting bin positions (RED_BIN, BLUE_BIN, REJECT_BIN)."
    )
    parser.add_argument("--port", default=SERIAL_PORT,
                        help=f"Serial port (default: {SERIAL_PORT})")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD,
                        help=f"Baud rate (default: {SERIAL_BAUD})")
    args = parser.parse_args()

    ser = None
    try:
        print("╔══════════════════════════════════════════════════════════════╗")
        print("║          BIN CALIBRATION — Sort Bin Positions               ║")
        print("╠══════════════════════════════════════════════════════════════╣")
        print("║  This tool calibrates the drop positions for each sorting   ║")
        print("║  bin (RED_BIN, BLUE_BIN, REJECT_BIN).                      ║")
        print("║                                                              ║")
        print("║  For each bin:                                               ║")
        print("║    1. Arm moves to the hardcoded bin position.              ║")
        print("║    2. Use WASD to fine-tune, or L for limp mode.            ║")
        print("║    3. Press ENTER to save the position.                     ║")
        print("║                                                              ║")
        print("║  Positions are saved to bin_calibration.json.               ║")
        print("╚══════════════════════════════════════════════════════════════╝\n")

        # ── Connect arm ────────────────────────────────────────────────
        try:
            ser = _open_serial(port=args.port, baud=args.baud)
        except Exception as e:
            print(f"❌ Could not connect to arm: {e}")
            return

        arm = ArmIK()

        # ── Move to scan pose initially ────────────────────────────────
        _move_to_scan_pose(ser)

        calibrated_bins: dict = {}
        total_bins = len(BIN_NAMES)

        for idx, bin_name in enumerate(BIN_NAMES):
            bx, by, bz = BINS[bin_name]

            print(f"\n{'━'*60}")
            print(f"  🗑️  BIN {idx + 1}/{total_bins}: {bin_name}")
            print(f"       Hardcoded position: X={bx:.1f}, Y={by:.1f}, Z={bz:.1f}")
            print(f"       Move the arm to the correct drop position for {bin_name}.")
            print(f"{'━'*60}")

            # Restore M3 full current for movement
            if M3_DEFAULT_CURRENT_LIMIT > 0:
                send_raw_command(ser, {"cmd": "set_current_limit", "id": 3, "value": M3_DEFAULT_CURRENT_LIMIT})

            # Move to home first, then to the bin starting position
            send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})
            send_ik_command(ser, arm, HOME_POSITION[0], HOME_POSITION[1], HOME_POSITION[2])
            time.sleep(0.5)

            # Enter WASD control loop starting at the hardcoded bin position
            result = _wasd_bin_phase(ser, arm, bin_name, idx, total_bins,
                                     start_x=bx, start_y=by, start_z=bz)

            if result is None:
                print("\n  ⛔ Calibration aborted by user.")
                return

            x, y, z, m4_off = result
            calibrated_bins[bin_name] = {
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(z, 2),
                "m4_offset": int(m4_off),
            }

            # Lift to clearance height and return to home
            send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})
            send_ik_command(ser, arm, x, y, CLEARANCE_HEIGHT)
            time.sleep(0.5)
            send_ik_command(ser, arm, HOME_POSITION[0], HOME_POSITION[1], HOME_POSITION[2])
            time.sleep(0.5)

        # ── Save all bin positions ─────────────────────────────────────
        _save_calibration(calibrated_bins)

        # ── Print summary table ────────────────────────────────────────
        print(f"\n{'═'*60}")
        print("  📋  BIN CALIBRATION SUMMARY")
        print(f"{'═'*60}")
        print(f"  {'Bin':<14} {'X':>7} {'Y':>7} {'Z':>7} {'m4_off':>8}")
        print(f"  {'─'*14} {'─'*7} {'─'*7} {'─'*7} {'─'*8}")
        for bn in BIN_NAMES:
            b = calibrated_bins[bn]
            print(f"  {bn:<14} {b['x']:>7.2f} {b['y']:>7.2f} {b['z']:>7.2f} {b['m4_offset']:>+8d}")
        print(f"{'═'*60}")

        # Return to scan pose
        _move_to_scan_pose(ser)
        print("\n  Done. Bin calibration complete! ✅\n")

    finally:
        if ser is not None:
            try:
                ser.close()
                print("\n🔌 Serial connection closed.")
            except Exception:
                pass


if __name__ == "__main__":
    main()
