#!/usr/bin/env python3
"""
verify_workspace.py
===================
Camera height and scan region verification — Step 6b.

Checks that the camera height matches config, calculates
worst-case parallax error, tests ball detection at 5 workspace
positions, and verifies IK reachability.  Works with or without
a camera connected (graceful fallback to manual confirmation).

Usage:
    python verify_workspace.py

Author: Bachelor Project 2026 – Autonomia
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import time
import math
from pathlib import Path


from config.arm import CAMERA_HEIGHT, SCAN_POSE
from ik.solver import ArmIK

# ── Serial settings (same as 06_homography.py) ───────────────────────
SERIAL_PORT = "/dev/cu.usbmodem2101"
SERIAL_BAUD = 115200

_ser = None  # lazily initialised on first call


# ── Serial helpers (pattern from 06_homography.py) ───────────────────

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

# ── Test positions: corners + centre of arm workspace ─────────────────
TEST_POSITIONS = [
    ("Centre",     20,   0),
    ("Near-left",  12, -15),
    ("Near-right", 12,  15),
    ("Far-left",   35, -15),
    ("Far-right",  35,  15),
]


# ══════════════════════════════════════════════════════════════════════
#  Phase helpers
# ══════════════════════════════════════════════════════════════════════

def _phase_camera_height() -> float:
    """Phase 1 — ask user for measured camera height and compare."""
    print("\n═══ Phase 1: Camera Height Verification ═══\n")
    print(f"  Measure from the desk surface to the camera lens (in cm).")
    print(f"  Current config value: CAMERA_HEIGHT = {CAMERA_HEIGHT} cm\n")

    while True:
        try:
            measured = float(input("  Measured camera height (cm): "))
            break
        except ValueError:
            print("  ❌ Invalid number, try again.")

    diff = abs(measured - CAMERA_HEIGHT)
    if diff < 1.0:
        status = "ok"
        print(f"  ✅ Camera height matches config (diff: {diff:.1f} cm)")
    elif diff < 3.0:
        status = "warn"
        print(f"  ⚠️  Camera height off by {diff:.1f} cm — may cause "
              f"~{diff * 0.07:.1f} cm parallax at workspace edges")
        print(f"     Consider updating CAMERA_HEIGHT to {measured:.1f} in src/config/arm.py")
    else:
        status = "fail"
        print(f"  ❌ Camera height off by {diff:.1f} cm — will cause "
              f"significant parallax error!")
        print(f"     UPDATE CAMERA_HEIGHT = {measured:.1f} in src/config/arm.py "
              f"before proceeding")

    return measured


def _phase_parallax(measured: float) -> bool:
    """Phase 2 — estimate worst-case parallax error."""
    print("\n═══ Phase 2: Parallax Estimate ═══")

    ball_height = 2.5          # cm, centre of a 50 mm ball
    max_offset = 25.0          # cm, approx max distance from camera axis

    if measured <= 0:
        print("\n  ❌ Invalid camera height — cannot compute parallax.")
        return False

    parallax_mm = (ball_height / measured) * max_offset * 10  # mm

    print(f"\n  Parallax estimate for 50 mm ball at workspace edge:")
    print(f"    Camera height:       {measured:.1f} cm")
    print(f"    Ball centre height:  {ball_height:.1f} cm")
    print(f"    Worst-case parallax: ~{parallax_mm:.1f} mm")

    if parallax_mm < 3:
        print(f"    ✅ Within acceptable range (< 3 mm)")
        return True
    else:
        print(f"    ⚠️  Parallax may affect accuracy — consider raising the camera")
        return False


def _phase_scan_coverage():
    """Phase 3 — check detection at workspace corners via camera or manual."""
    print("\n═══ Phase 3: Scan Region Coverage ═══\n")
    print("  This test checks that the camera can detect balls at the edges")
    print("  of the arm's reachable workspace.\n")
    print(f"  You'll place a ball at {len(TEST_POSITIONS)} test positions one at a time.\n")

    # ── Try to start VisionBridge ─────────────────────────────────────
    vision = None
    camera_available = False
    try:
        from ik.vision_bridge import VisionBridge
        vision = VisionBridge(use_camera=True)
        if vision.open():
            camera_available = True
            print("  ✅ Camera connected — using automated detection.\n")
        else:
            print("  ⚠️  Camera could not be opened.")
            print("  Skipping automated detection — switching to manual mode.\n")
    except Exception as e:
        print(f"  ⚠️  Could not start camera: {e}")
        print("  Skipping automated detection — switching to manual mode.\n")

    # ── Detection loop ────────────────────────────────────────────────
    detected_count = 0
    total = len(TEST_POSITIONS)

    for name, x, y in TEST_POSITIONS:
        print(f"  Position: {name} (X={x}, Y={y} cm)")
        input(f"    Place a ball at approximately X={x}, Y={y} cm. "
              f"Press ENTER when ready...")

        if camera_available and vision is not None:
            # Try detection — wait up to 3 seconds (6 × 0.5 s)
            detected = False
            for _attempt in range(6):
                try:
                    detections = vision.scan_for_balls(num_frames=3)
                except Exception:
                    detections = []

                if detections:
                    for det in detections:
                        dx, dy = det["x"], det["y"]
                        dist = math.sqrt((dx - x) ** 2 + (dy - y) ** 2)
                        if dist < 10:  # within 10 cm of expected
                            print(f"    ✅ Detected at ({dx:.1f}, {dy:.1f}) cm "
                                  f"— offset: {dist:.1f} cm")
                            detected = True
                            break
                if detected:
                    break
                time.sleep(0.5)

            if detected:
                detected_count += 1
            else:
                print(f"    ❌ NOT detected within 3 seconds")
        else:
            visible = input("    Can you see the ball in the camera "
                            "feed? (y/n): ").strip().lower()
            if visible == "y":
                print(f"    ✅ Ball visible at {name}")
                detected_count += 1
            else:
                print(f"    ❌ Ball NOT visible at {name}")

    # ── Cleanup camera ────────────────────────────────────────────────
    if vision is not None:
        try:
            vision.close()
        except Exception:
            pass

    return detected_count, total


def _phase_ik_reachability():
    """Check IK reachability for every test position."""
    print("\n═══ Phase 4: IK Reachability Check ═══\n")

    arm = ArmIK()
    reachable_count = 0
    total = len(TEST_POSITIONS)

    for name, x, y in TEST_POSITIONS:
        try:
            arm.solve(x, y, 2.0)  # z = 2 cm — grab height near table
            print(f"    IK: ✅ ({name}) reachable")
            reachable_count += 1
        except Exception:
            print(f"    IK: ❌ ({name}) NOT reachable — adjust workspace bounds")

    return reachable_count, total


# ══════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════

def main():
    print("╔═══════════════════════════════════════════════════════════╗")
    print("║        WORKSPACE VERIFICATION — Step 6b                  ║")
    print("╠═══════════════════════════════════════════════════════════╣")
    print("║  Verifies camera height and scan region coverage.        ║")
    print("║                                                          ║")
    print("║  You'll need: a ruler, a coloured ball, and both the     ║")
    print("║  camera and arm connected.                               ║")
    print("╚═══════════════════════════════════════════════════════════╝")

    # ── Step 0: Connect to OpenRB-150 and move arm to SCAN_POSE ───────
    print("\n[INIT] Connecting to OpenRB-150 and moving arm to SCAN_POSE...")
    try:
        _move_to_scan_pose()
        if _ser is not None:
            _ser.close()
    except Exception as e:
        err_str = str(e)
        if ("No such file or directory" in err_str
                or "Errno 2" in err_str
                or "could not open port" in err_str.lower()):
            print(f"\n  ⚠️  Serial port not found: {e}")
            print(f"       (Expected port: {SERIAL_PORT})")
            print("       Make sure the arm is manually moved to SCAN_POSE.")
            ans = input("  Is the arm already at SCAN_POSE? (y/n): ").strip().lower()
            if ans != 'y':
                print("  ❌ Please move the arm to SCAN_POSE and re-run this script.")
                return
            print("  [ARM] ✅ Arm is at SCAN_POSE (confirmed by user).")
        else:
            print(f"\n  ❌ Could not connect to OpenRB-150 or move arm: {e}")
            print("     Check serial connection and try again.")
            return

    # ── Phase 1: Camera height ────────────────────────────────────────
    measured = _phase_camera_height()

    # ── Phase 2: Parallax estimate ────────────────────────────────────
    parallax_ok = _phase_parallax(measured)

    # ── Phase 3: Scan region coverage ─────────────────────────────────
    detected, det_total = _phase_scan_coverage()

    # ── Phase 4: IK reachability ──────────────────────────────────────
    reachable, ik_total = _phase_ik_reachability()

    # ── Summary ───────────────────────────────────────────────────────
    diff = abs(measured - CAMERA_HEIGHT)
    if diff < 1.0:
        height_status = "✅ OK"
    elif diff < 3.0:
        height_status = "⚠️  Minor offset"
    else:
        height_status = "❌ MISMATCH"

    parallax_status = "✅ OK" if parallax_ok else "⚠️  High"

    print(f"\n{'═' * 59}")
    print("  Summary")
    print(f"{'═' * 59}\n")
    print(f"  Camera height:   {height_status}")
    print(f"  Parallax:        {parallax_status}")
    print(f"  Scan coverage:   {detected}/{det_total} positions detected")
    print(f"  IK reachability: {reachable}/{ik_total} positions reachable")

    all_pass = (diff < 3.0
                and parallax_ok
                and detected == det_total
                and reachable == ik_total)

    print()
    if all_pass:
        print("  🎉  ALL CHECKS PASSED — workspace is ready for sorting.")
    else:
        print("  ⚠️  Some checks failed — review the output above and fix")
        print("     any issues before running the main sorting pipeline.")

    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n  ⛔ Interrupted — exiting.\n")
        sys.exit(1)
