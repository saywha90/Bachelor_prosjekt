#!/usr/bin/env python3
"""
calibrate_claw.py
=================
Claw (gripper) open/close calibration — Step 2b.

Interactively tunes the m5 motor positions for fully open and
firmly gripping a 50 mm ball.  Tests jaw symmetry and runs
open/close cycles.  Saves results to claw_calibration.json.

Includes adaptive grip testing: after calibration values are found,
you can verify grip behaviour with current-limit safety, load
polling, and stall detection — identical to production main.py.

Usage:
    python calibrate_claw.py

Author: Bachelor Project 2026 – Autonomia
"""


from __future__ import annotations

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import os
import tempfile
import time
import traceback
from pathlib import Path


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

from ik.solver import ArmIK
from config.arm import (
    GRIP_CURRENT_LIMIT, GRIP_PROFILE_VEL, GRIP_PROFILE_ACC,
    GRIP_POLL_INTERVAL, GRIP_TIMEOUT, GRIP_LOAD_DETECT,
    GRIP_POSITION_STALL, GRIP_EXTRA_CLOSE, GRIP_VERIFY_TOLERANCE,
    GRAB_DWELL, M5_DEFAULT_CURRENT_LIMIT, CLAW_OPEN_POS,
)

# ── Serial wrapper ──────────────────────────────────────────────────
# Identical lazy-singleton pattern used by calibrate_joints.py and
# calibrate_sag.py.  Change SERIAL_PORT / SERIAL_BAUD to match your
# setup.

SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD = 115200

# Calibration-safe motion profile (slower than production defaults)
CAL_PROFILE_VEL = 40
CAL_PROFILE_ACC = 10

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


def send_raw_command(cmd_dict: dict) -> str:
    """Send an arbitrary JSON command and return the firmware response.

    Unlike ``send_command()`` (which sends motor goal positions), this
    sends firmware meta-commands such as ``set_current_limit``,
    ``set_profile``, ``read_load``, etc.
    """
    ser = _get_serial()
    ser.write((json.dumps(cmd_dict) + "\n").encode())
    try:
        return ser.readline().decode(errors="replace").strip()
    except Exception:
        return ""


def read_load() -> dict | None:
    """Read present load from all motors.

    Returns a dict like {"m1": …, "m5": …} or ``None`` on failure.
    """
    ser = _get_serial()
    ser.write((json.dumps({"cmd": "read_load"}) + "\n").encode())
    try:
        return json.loads(ser.readline().decode(errors="replace").strip())
    except (json.JSONDecodeError, TypeError):
        return None


# ── Adaptive grip ───────────────────────────────────────────────────

def adaptive_grip(claw_closed: int) -> bool:
    """Attempt an adaptive grip toward *claw_closed* with current-limit
    safety, load polling, and stall detection — identical logic to
    main.py's ``adaptive_grip()`` but wired into the calibration helpers.

    Returns ``True`` if an object was gripped, ``False`` if closed on air.
    """
    print("  🤏 Starting adaptive grip sequence…")
    try:
        # 1. Set M5 current limit to safe value
        print(f"  🤏 Setting M5 current limit to {GRIP_CURRENT_LIMIT} mA")
        send_raw_command({"cmd": "set_current_limit", "id": 5, "value": GRIP_CURRENT_LIMIT})

        # 2. Set slow motion profile
        print(f"  🤏 Setting slow profile (vel={GRIP_PROFILE_VEL}, acc={GRIP_PROFILE_ACC})")
        send_raw_command({"cmd": "set_profile", "vel": GRIP_PROFILE_VEL, "acc": GRIP_PROFILE_ACC})

        # 3. Read current positions, command M5 to close
        positions = read_positions()
        if positions is None:
            print("  ⚠️  Cannot read positions — aborting adaptive grip")
            return False

        prev_pos = int(positions.get("m5", 0))
        goal = {k: int(v) for k, v in positions.items() if k.startswith("m")}
        goal["m5"] = claw_closed
        send_command(goal)

        # 4. Poll loop — detect contact via load or stall
        contact = False
        contact_position = None
        start = time.time()

        while True:
            time.sleep(GRIP_POLL_INTERVAL)
            elapsed = time.time() - start
            if elapsed > GRIP_TIMEOUT:
                print(f"  ⏱ Grip timeout after {GRIP_TIMEOUT}s")
                break

            loads = read_load()
            cur_positions = read_positions()
            if loads is None or cur_positions is None:
                continue

            m5_load = abs(int(loads.get("m5", 0)))
            m5_pos = int(cur_positions.get("m5", 0))

            # Contact via load
            if m5_load >= GRIP_LOAD_DETECT:
                print(f"  ✅ Contact detected via load ({m5_load} ≥ {GRIP_LOAD_DETECT})")
                contact = True
                contact_position = m5_pos
                break

            # Stall detection
            if abs(m5_pos - prev_pos) <= GRIP_POSITION_STALL:
                print(f"  ✅ Stall detected (Δpos={abs(m5_pos - prev_pos)} ≤ {GRIP_POSITION_STALL})")
                contact = True
                contact_position = m5_pos
                break

            prev_pos = m5_pos

        # 5. If contact, still command the configured closed target and give
        #    the servo time to finish/settle before reporting success.  The
        #    ball may block the jaws physically, but the command must be the
        #    calibrated close target so calibration does not validate the old
        #    half-close-before-lift behaviour.
        if contact and contact_position is not None:
            cur_positions = read_positions()
            if cur_positions is not None:
                extra_goal = {k: int(v) for k, v in cur_positions.items() if k.startswith("m")}
                extra_goal["m5"] = claw_closed
                send_command(extra_goal)
                time.sleep(max(GRAB_DWELL, GRIP_POLL_INTERVAL * 3))
            print(f"  🔧 Secure close command: {contact_position} → {claw_closed}")

        # 6. Final position check — closed on air?
        final_positions = read_positions()
        if final_positions and "m5" in final_positions:
            final_m5 = int(final_positions["m5"])
            if abs(final_m5 - claw_closed) <= GRIP_VERIFY_TOLERANCE:
                print(f"  ⚠️ Closed on air (final pos {final_m5} ≈ target {claw_closed})")
                return False
            print(f"  ✅ Object gripped at position {final_m5}")
            return True

        print("  ⚠️  Cannot read final position — assuming no grip")
        return False

    finally:
        # 7. ALWAYS restore defaults
        # Flush any partial serial data left by an interrupted I/O (e.g. Ctrl+C)
        try:
            ser = _get_serial()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass
        try:
            send_raw_command({"cmd": "set_current_limit", "id": 5, "value": M5_DEFAULT_CURRENT_LIMIT})
        except Exception as e:
            print(f"  ⚠️  Failed to restore M5 current limit: {e}")
        # Restore the calibration-safe profile (vel=40, acc=10), NOT the
        # production defaults (vel=80, acc=20) — the operator's hands may
        # be near the mechanism during calibration.
        try:
            send_raw_command({"cmd": "set_profile", "vel": CAL_PROFILE_VEL, "acc": CAL_PROFILE_ACC})
        except Exception as e:
            print(f"  ⚠️  Failed to restore profile: {e}")
        print("  🔄 Restored default current limit and calibration profile")


# ── Constants ───────────────────────────────────────────────────────
NEUTRAL = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911, "m5": 2048}

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

        claw_open = tune_m5(safe_pos, CLAW_OPEN_POS, "CLAW_OPEN")
        print(f"\n  ✅ CLAW_OPEN = {claw_open}")

        # ════════════════════════════════════════════════════════════
        # Phase 2 — Find CLAW_CLOSED position (EMPTY)
        # ════════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 2 — Find CLAW_CLOSED position (EMPTY)")
        print("=" * 60)
        print("Make sure the claw is EMPTY (remove the ball).")
        input("Press ENTER when the claw is empty... ")

        print("\nAdjust m5 to close the claw until the jaws just touch each other.")
        print("This is the 'closed on air' position used to detect if a pick failed.")

        claw_closed = tune_m5({**safe_pos, "m5": claw_open}, claw_open, "CLAW_CLOSED")
        print(f"\n  ✅ CLAW_CLOSED = {claw_closed}")

        # ════════════════════════════════════════════════════════════
        # Phase 2b — Adaptive grip test (optional)
        # ════════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 2b — Adaptive grip test")
        print("=" * 60)
        print("Test the adaptive grip with your calibrated closed position.")
        print("This uses current-limit safety, load polling, and stall")
        print("detection — identical to production main.py.")

        while True:
            answer = input("\nPlace a ball in the gripper and press ENTER to test "
                           "(or type 'skip' to skip)… ").strip().lower()
            if answer == "skip":
                print("  ⏭️  Skipping adaptive grip test")
                break

            # Open claw first
            print("  Opening claw…")
            goto({**safe_pos, "m5": claw_open}, pause_s=1.0)

            print("  Running adaptive grip…")
            grip_ok = adaptive_grip(claw_closed)
            print(f"  Result: gripped={grip_ok}")

            again = input("  Try again? (y/n): ").strip().lower()
            if again not in ("y", "yes"):
                break

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
        # Phase 4 — Cycle test (with adaptive grip)
        # ════════════════════════════════════════════════════════════
        print("\n" + "=" * 60)
        print("PHASE 4 — Cycle test (3 open/close cycles with adaptive grip)")
        print("=" * 60)
        print("Running 3 open/close cycles with adaptive grip to verify…")
        print("Place a ball in the gripper before starting.")
        input("Press ENTER when ready… ")

        for i in range(1, 4):
            print(f"\n  Cycle {i}/3 — opening…")
            goto({**safe_pos, "m5": claw_open}, pause_s=1.0)
            print(f"  Cycle {i}/3 — adaptive grip closing…")
            grip_ok = adaptive_grip(claw_closed)
            print(f"  Cycle {i}: grip_ok={grip_ok}")

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
        _save_json_atomic(out_path, calibration)

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
    except Exception:
        traceback.print_exc()
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
