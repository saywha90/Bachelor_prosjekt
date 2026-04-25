#!/usr/bin/env python3
"""
calibrate_claw.py
=================
Claw (gripper) open/close calibration — Step 2b.

Interactively tunes the m5 motor positions for fully open and
firmly gripping a 50 mm ball.  Tests jaw symmetry and runs
open/close cycles.  Saves results to claw_calibration.json.

Usage:
    python calibrate_claw.py

Author: Bachelor Project 2026 – Autonomia
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import time
from pathlib import Path

from ik.solver import ArmIK

# ── Serial wrapper ──────────────────────────────────────────────────
# Identical lazy-singleton pattern used by calibrate_joints.py and
# calibrate_sag.py.  Change SERIAL_PORT / SERIAL_BAUD to match your
# setup.

SERIAL_PORT = "/dev/cu.usbmodem2101"
SERIAL_BAUD = 115200

_ser = None  # lazily initialised on first call


def _get_serial():
    """Return the shared serial connection, opening it on first use."""
    global _ser
    if _ser is not None:
        return _ser
    import serial
    print(f"[SERIAL] Opening {SERIAL_PORT} @ {SERIAL_BAUD} …")
    _ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=2)
    time.sleep(3)  # wait for OpenRB-150 to boot

    # Drain any boot messages
    while _ser.in_waiting:
        _ser.readline()

    # Re-enable torque (mirrors main.py smooth_startup)
    _ser.write((json.dumps({"cmd": "enable_torque"}) + "\n").encode())
    time.sleep(0.5)
    _ser.readline()

    # Set a conservative motion profile so large jumps are slow
    _ser.write((json.dumps({"cmd": "set_profile", "vel": 40, "acc": 10}) + "\n").encode())
    time.sleep(0.3)
    _ser.readline()
    print("[SERIAL] Ready (profile: vel=40, acc=10)")
    return _ser


# ── Helpers ─────────────────────────────────────────────────────────

def send_command(positions: dict):
    """Send a dict of motor positions (e.g. {"m1": 2048, …}) to the OpenRB.

    The firmware expects a JSON object with keys m1–m5 containing
    Dynamixel step values (0–4095).  Returns the firmware response string.
    """
    ser = _get_serial()
    ser.write((json.dumps(positions) + "\n").encode())
    time.sleep(0.1)
    resp = ser.readline().decode(errors="replace").strip()
    return resp


def goto(positions: dict, pause_s: float = 2.0):
    """Send absolute motor targets and wait for motion to finish."""
    send_command(positions)
    time.sleep(pause_s)


def read_positions() -> dict:
    """Ask the firmware for the current motor positions.

    Returns a dict like {"m1": 2048, "m2": …, "m5": …} or ``None``
    on failure.
    """
    ser = _get_serial()
    cmd = json.dumps({"cmd": "read_pos"})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode(errors="replace").strip()
    try:
        return json.loads(resp)
    except (json.JSONDecodeError, TypeError):
        print(f"  ⚠️  Could not parse position response: {resp}")
        return None


# ── Constants ───────────────────────────────────────────────────────
NEUTRAL = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 2048, "m5": 2048}

M5_MIN = 500
M5_MAX = 3500


# ── Interactive m5 tuning loop ──────────────────────────────────────

def tune_m5(current_positions: dict, start_value: int, header: str) -> int:
    """Interactive loop for tuning the m5 (claw) position.

    *current_positions* is the full motor dict (m1–m5) used as the
    baseline; only m5 is modified.  Returns the chosen m5 value.
    """
    value = start_value
    print(f"\n  Current m5 position: {value}")

    while True:
        raw = input(
            f"\n  Enter new m5 value to test ({M5_MIN}-{M5_MAX}), or:\n"
            f"    '+'  to increase by 50\n"
            f"    '-'  to decrease by 50\n"
            f"    '++' to increase by 200\n"
            f"    '--' to decrease by 200\n"
            f"    'done' when satisfied\n"
            f"  Current m5: {value}\n"
            f"  > "
        ).strip()

        if raw.lower() == "done":
            return value

        if raw == "++":
            value += 200
        elif raw == "--":
            value -= 200
        elif raw == "+":
            value += 50
        elif raw == "-":
            value -= 50
        else:
            try:
                value = int(raw)
            except ValueError:
                print("  ⚠️  Invalid input — enter a number, +, -, ++, --, or done")
                continue

        # Clamp to safe range
        if value < M5_MIN or value > M5_MAX:
            value = max(M5_MIN, min(M5_MAX, value))
            print(f"  ⚠️  Clamped to safe range [{M5_MIN}, {M5_MAX}]: m5 = {value}")

        # Apply the new m5 position
        updated = {**current_positions, "m5": value}
        send_command(updated)
        time.sleep(0.5)
        print(f"  → m5 set to {value}")


# ── Main ────────────────────────────────────────────────────────────

def main():
    # ── Banner ──────────────────────────────────────────────────────
    print("╔═══════════════════════════════════════════════════════════╗")
    print("║            CLAW (GRIPPER) CALIBRATION — Step 2b          ║")
    print("╠═══════════════════════════════════════════════════════════╣")
    print("║  This script helps you find the correct m5 positions     ║")
    print("║  for opening and closing the claw.                       ║")
    print("║                                                          ║")
    print("║  You'll need: a 50mm ball (or the object to grip)        ║")
    print("╚═══════════════════════════════════════════════════════════╝")
    print()
    input("Clear the workspace around the arm, then press ENTER to start... ")

    symmetric = False
    cycle_test_passed = False

    try:
        # ── Move to neutral ─────────────────────────────────────────
        print("\nMoving to NEUTRAL position…")
        goto(NEUTRAL, pause_s=3.0)

        # ── Move to a comfortable observation pose via IK ───────────
        print("Moving to a safe grab observation pose (20, 0, 15)…")
        arm = ArmIK()
        safe_pos = arm.solve(20, 0, 15)
        goto(safe_pos, pause_s=2.5)

        # ════════════════════════════════════════════════════════════
        # Phase 1 — Find CLAW_OPEN position
        # ════════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 1 — Find CLAW_OPEN position")
        print("=" * 60)
        print("Adjust m5 until the claw is fully open without straining")
        print("the motor (no buzzing / high current draw).")

        claw_open = tune_m5(safe_pos, 2048, "CLAW_OPEN")
        print(f"\n  ✅ CLAW_OPEN = {claw_open}")

        # ════════════════════════════════════════════════════════════
        # Phase 2 — Find CLAW_CLOSED position
        # ════════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 2 — Find CLAW_CLOSED position")
        print("=" * 60)
        print("Place the ball between the claw jaws now.")
        input("Press ENTER when the ball is in position... ")

        print("\nAdjust m5 to close on the ball. It should grip firmly")
        print("but not stall the motor.")

        claw_closed = tune_m5({**safe_pos, "m5": claw_open}, claw_open, "CLAW_CLOSED")
        print(f"\n  ✅ CLAW_CLOSED = {claw_closed}")

        # ════════════════════════════════════════════════════════════
        # Phase 3 — Symmetry check
        # ════════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 3 — Symmetry check")
        print("=" * 60)

        # Open the claw
        print("Opening claw…")
        goto({**safe_pos, "m5": claw_open}, pause_s=1.5)
        print("Watch from the SIDE: remove the ball, then we'll close again.")
        input("Press ENTER when the ball is removed... ")

        # Close the claw
        print("Closing claw…")
        goto({**safe_pos, "m5": claw_closed}, pause_s=1.5)

        answer = input("\nDid both jaws close evenly? (y/n): ").strip().lower()
        symmetric = answer in ("y", "yes")
        if not symmetric:
            print("\n  ⚠️  The claw may be off-centre.")
            print("  Consider adjusting L3 (wrist-to-claw offset) in")
            print("  src/ik/solver.py or adding a small Y offset in src/config/arm.py.")

        # ════════════════════════════════════════════════════════════
        # Phase 4 — Cycle test
        # ════════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 4 — Cycle test (3 open/close cycles)")
        print("=" * 60)
        print("Running 3 open/close cycles to verify…")

        for i in range(1, 4):
            print(f"  Cycle {i}/3 — opening…")
            goto({**safe_pos, "m5": claw_open}, pause_s=1.0)
            print(f"  Cycle {i}/3 — closing…")
            goto({**safe_pos, "m5": claw_closed}, pause_s=1.0)

        answer = input("\nDid all 3 cycles work cleanly? (y/n): ").strip().lower()
        cycle_test_passed = answer in ("y", "yes")

        # ════════════════════════════════════════════════════════════
        # Save results
        # ════════════════════════════════════════════════════════════
        calibration = {
            "claw_open_pos": claw_open,
            "claw_closed_pos": claw_closed,
            "symmetric": symmetric,
            "cycle_test_passed": cycle_test_passed,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }

        out_path = Path(__file__).resolve().parent.parent / "ik" / "claw_calibration.json"
        with open(out_path, "w") as f:
            json.dump(calibration, f, indent=2)

        print()
        print("┌──────────────────────────────────────────┐")
        print("│  Update these values in src/config/arm.py:│")
        print("│                                          │")
        print(f"│  CLAW_OPEN_POS  = {claw_open:<22}│")
        print(f"│  CLAW_CLOSED_POS = {claw_closed:<21}│")
        print("│                                          │")
        print("│  Also update in main.py lines 53-54:     │")
        print(f"│  CLAW_OPEN_POS  = {claw_open:<22}│")
        print(f"│  CLAW_CLOSED_POS = {claw_closed:<21}│")
        print("└──────────────────────────────────────────┘")
        print(f"\nSaved to {out_path}")

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
    except Exception as e:
        print(f"\n⚠️  Error: {e}")
    finally:
        # ── Return to neutral ───────────────────────────────────────
        print("\nReturning to NEUTRAL position…")
        try:
            goto(NEUTRAL, pause_s=2.0)
        except Exception:
            pass

    print("\n" + "=" * 60)
    print("DONE.")
    print("=" * 60)


if __name__ == "__main__":
    main()
