#!/usr/bin/env python3
"""
11_claw_grip_test.py
====================
Claw grip diagnostic / calibration tool — Step 11.

Moves the arm to a neutral position, then runs repeated grip tests
while logging all sensor values (position, load, current).  Reports
a clear GRIP DETECTED / NO GRIP DETECTED verdict using the same
thresholds as production ``main.py``.

Usage:
    python 11_claw_grip_test.py                  # auto-detect serial port
    python 11_claw_grip_test.py /dev/cu.usbmodem101  # explicit port

Author: Bachelor Project 2026 – Autonomia
"""

from __future__ import annotations

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import time
import traceback

from config.arm import (
    CLAW_OPEN_POS,
    CLAW_CLOSED_POS,
    GRIP_CURRENT_LIMIT,
    GRIP_PROFILE_VEL,
    GRIP_PROFILE_ACC,
    GRIP_POLL_INTERVAL,
    GRIP_TIMEOUT,
    GRIP_LOAD_DETECT,
    GRIP_LOAD_THRESHOLD,
    GRIP_POSITION_STALL,
    GRIP_EXTRA_CLOSE,
    GRIP_MIN_BALL_BLOCKED_STEPS,
    GRIP_MIN_BLOCKED_WITH_SENSOR,
    GRIP_VERIFY_TOLERANCE,
    DEFAULT_PROFILE_VEL,
    DEFAULT_PROFILE_ACC,
    M5_DEFAULT_CURRENT_LIMIT,
    EXPECTED_BALL_DIAMETER_CM,
    SCAN_POSE,
)

# ── Serial settings ─────────────────────────────────────────────────
SERIAL_BAUD = 115200
CAL_PROFILE_VEL = 40
CAL_PROFILE_ACC = 10

# Neutral position — shoulder raised so the claw is accessible
NEUTRAL = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911, "m5": CLAW_OPEN_POS}

_ser = None


# ── Port auto-detection ─────────────────────────────────────────────
def _find_serial_port() -> str:
    """Try to auto-detect the OpenRB-150 serial port."""
    import glob

    candidates = (
        glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
    )
    if candidates:
        return candidates[0]
    return "/dev/cu.usbmodem101"


def _get_serial(port: str | None = None):
    """Return the shared serial connection, opening it on first use."""
    global _ser
    if _ser is not None:
        return _ser

    import serial as pyserial

    if port is None:
        port = _find_serial_port()

    print(f"[INFO] Opening {port} @ {SERIAL_BAUD} …")
    _ser = pyserial.Serial(port, SERIAL_BAUD, timeout=2)
    time.sleep(3)  # wait for OpenRB-150 to boot

    # Drain boot messages
    while _ser.in_waiting:
        _ser.readline()

    # Enable torque
    _ser.write((json.dumps({"cmd": "enable_torque"}) + "\n").encode())
    time.sleep(0.5)
    _ser.readline()

    # Set conservative motion profile
    _ser.write(
        (json.dumps({"cmd": "set_profile", "vel": CAL_PROFILE_VEL, "acc": CAL_PROFILE_ACC}) + "\n").encode()
    )
    time.sleep(0.3)
    _ser.readline()
    print(f"[INFO] Connected to {port}")
    return _ser


# ── Low-level helpers ────────────────────────────────────────────────

def send_raw(cmd_dict: dict) -> str:
    """Send an arbitrary JSON command and return the firmware response."""
    ser = _get_serial()
    ser.write((json.dumps(cmd_dict) + "\n").encode())
    try:
        return ser.readline().decode(errors="replace").strip()
    except Exception:
        return ""


def send_positions(positions: dict) -> str:
    """Send motor goal positions (m1–m5)."""
    ser = _get_serial()
    ser.write((json.dumps(positions) + "\n").encode())
    time.sleep(0.1)
    return ser.readline().decode(errors="replace").strip()


def goto(positions: dict, pause_s: float = 2.0):
    """Send absolute motor targets and wait for motion to finish."""
    send_positions(positions)
    time.sleep(pause_s)


def read_positions() -> dict | None:
    """Read current motor positions from firmware."""
    ser = _get_serial()
    ser.write((json.dumps({"cmd": "read_pos"}) + "\n").encode())
    try:
        return json.loads(ser.readline().decode(errors="replace").strip())
    except (json.JSONDecodeError, TypeError):
        return None


def read_load() -> dict | None:
    """Read present load from all motors."""
    ser = _get_serial()
    ser.write((json.dumps({"cmd": "read_load"}) + "\n").encode())
    try:
        return json.loads(ser.readline().decode(errors="replace").strip())
    except (json.JSONDecodeError, TypeError):
        return None


def read_current() -> dict | None:
    """Read present current from all motors."""
    ser = _get_serial()
    ser.write((json.dumps({"cmd": "read_current"}) + "\n").encode())
    try:
        return json.loads(ser.readline().decode(errors="replace").strip())
    except (json.JSONDecodeError, TypeError):
        return None


def read_claw_feedback(fallback_position: int | None = None) -> dict:
    """Read M5 position/load/current, tolerating unavailable sensors."""
    loads = read_load()
    currents = read_current()
    positions = read_positions()

    position = None
    if positions and "m5" in positions:
        position = int(positions["m5"])
    elif fallback_position is not None:
        position = int(fallback_position)

    load = abs(int(loads.get("m5", 0))) if loads and "m5" in loads else None
    current_ma = abs(int(currents.get("m5", 0))) if currents and "m5" in currents else None
    return {
        "position": position,
        "load": load,
        "current_ma": current_ma,
    }


# ── Grip analysis helpers (mirrors main.py logic) ───────────────────

def _claw_close_direction() -> int:
    return 1 if CLAW_CLOSED_POS >= CLAW_OPEN_POS else -1


def _clamp_claw_position(position: int) -> int:
    low = min(CLAW_OPEN_POS, CLAW_CLOSED_POS)
    high = max(CLAW_OPEN_POS, CLAW_CLOSED_POS)
    return max(low, min(high, int(position)))


def _claw_reached_closed(position: int) -> bool:
    direction = _claw_close_direction()
    return (int(position) - CLAW_CLOSED_POS) * direction >= 0


def _claw_blocked_from_closed(position: int) -> int:
    return abs(CLAW_CLOSED_POS - int(position))


def _grip_current_contact_threshold() -> int:
    return max(20, int(GRIP_CURRENT_LIMIT * 0.4))


def _claw_position_indicates_5cm_ball(position: int) -> bool:
    return _claw_blocked_from_closed(position) > GRIP_MIN_BALL_BLOCKED_STEPS


def feedback_confirms_grip(feedback: dict) -> bool:
    """Two-tier grip confirmation — same logic as main.py._feedback_confirms_grip()."""
    # Safety check: zero load + near-zero current = definitely no ball
    # (the claw may not have finished closing)
    load_val = feedback.get("load", 0) or 0
    current_val = feedback.get("current_ma", 0) or 0
    if load_val <= 0 and current_val < 5:
        print(f"  [GRIP] ❌ No grip: load=0 and current={current_val} mA — no resistance detected")
        return False

    position = feedback.get("position")
    blocked_steps = 0
    if position is not None:
        blocked_steps = _claw_blocked_from_closed(position)

    minimally_blocked = blocked_steps >= GRIP_MIN_BLOCKED_WITH_SENSOR
    strongly_blocked = position is not None and _claw_position_indicates_5cm_ball(position)

    # Load check
    load = feedback.get("load")
    if load is not None and load >= GRIP_LOAD_THRESHOLD:
        if minimally_blocked:
            return True

    # Current check
    current_ma = feedback.get("current_ma")
    current_threshold = _grip_current_contact_threshold()
    if current_ma is not None and current_ma >= current_threshold:
        if minimally_blocked:
            return True

    # Position-only check
    if position is None:
        return False
    if strongly_blocked:
        return True

    return False


# ── Adaptive grip test with full logging ─────────────────────────────

# Generous timeout for the incremental close loop.  With ~18 increments
# of 30 steps each and 2 sensor reads per increment at ~0.2 s, the bare
# minimum is ~7.5 s.  We use 15.0 s to ensure the loop never times out
# before the claw physically reaches CLAW_CLOSED_POS or stalls.
_DIAGNOSTIC_CLOSE_TIMEOUT = 15.0

# Settle time after the secure close command — the motor needs this long
# to actually reach CLAW_CLOSED_POS (or stall against a ball).
_SECURE_CLOSE_SETTLE = 3.0


def run_grip_test(base_positions: dict) -> dict:
    """Execute one incremental adaptive grip with detailed sensor logging.

    Returns a result dict with all feedback and the grip verdict.
    """
    goal = base_positions.copy()
    direction = _claw_close_direction()
    travel = _claw_blocked_from_closed(CLAW_OPEN_POS)
    close_step = max(1, min(abs(GRIP_EXTRA_CLOSE), travel))

    # 1. Set M5 current limit
    print(f"\n[GRIP TEST] Setting M5 current limit to {GRIP_CURRENT_LIMIT} mA")
    send_raw({"cmd": "set_current_limit", "id": 5, "value": GRIP_CURRENT_LIMIT})

    # 2. Set slow profile
    print(f"[GRIP TEST] Setting slow profile (vel={GRIP_PROFILE_VEL}, acc={GRIP_PROFILE_ACC})")
    send_raw({"cmd": "set_profile", "vel": GRIP_PROFILE_VEL, "acc": GRIP_PROFILE_ACC})

    # 3. Ensure claw is at open position
    goal["m5"] = CLAW_OPEN_POS
    send_positions(goal)
    time.sleep(0.5)

    # 4. Incremental close with polling
    close_timeout = _DIAGNOSTIC_CLOSE_TIMEOUT
    print(f"\n[GRIP TEST] Starting adaptive close… (timeout={close_timeout:.1f}s)")
    print(f"  Open={CLAW_OPEN_POS} → Close={CLAW_CLOSED_POS} in {close_step}-step increments\n")

    current_target = CLAW_OPEN_POS
    prev_pos = CLAW_OPEN_POS
    contact_detected = False
    contact_position = None
    contact_reason = None
    incremental_outcome = "unknown"   # "contact", "reached_limit", "timeout"
    stall_reads = 0
    start_time = time.time()

    while not _claw_reached_closed(current_target):
        elapsed = time.time() - start_time
        if elapsed > close_timeout:
            print(f"  ⏱ Timeout after {close_timeout:.1f}s (pos≈{prev_pos})")
            incremental_outcome = "timeout"
            break

        old_target = current_target
        current_target = _clamp_claw_position(current_target + direction * close_step)
        goal["m5"] = current_target
        send_positions(goal)

        step_start = time.time()
        while True:
            time.sleep(GRIP_POLL_INTERVAL)
            elapsed = time.time() - start_time

            if elapsed > close_timeout:
                print(f"  ⏱ Timeout after {close_timeout:.1f}s (pos≈{prev_pos})")
                incremental_outcome = "timeout"
                break

            loads = read_load()
            currents = read_current()
            cur_positions = read_positions()

            if cur_positions is None:
                continue

            m5_load = abs(int(loads.get("m5", 0))) if loads else 0
            m5_current = abs(int(currents.get("m5", 0))) if currents else 0
            m5_pos = int(cur_positions.get("m5", prev_pos))
            blocked = _claw_blocked_from_closed(m5_pos)

            # Print step detail
            marker = ""
            if m5_load >= GRIP_LOAD_DETECT:
                marker = " ← CONTACT (load)"
            elif m5_current >= _grip_current_contact_threshold():
                marker = " ← CONTACT (current)"
            print(
                f"  Step {old_target} → {current_target}: "
                f"pos={m5_pos}, load={m5_load}, current={m5_current} mA, "
                f"blocked={blocked}{marker}"
            )

            # Contact via load
            if m5_load >= GRIP_LOAD_DETECT:
                contact_detected = True
                contact_position = m5_pos
                contact_reason = "load"
                incremental_outcome = "contact"
                break

            # Contact via current
            if m5_current >= _grip_current_contact_threshold():
                contact_detected = True
                contact_position = m5_pos
                contact_reason = "current"
                incremental_outcome = "contact"
                break

            # Reached incremental target — advance
            if abs(m5_pos - current_target) <= GRIP_POSITION_STALL:
                prev_pos = m5_pos
                stall_reads = 0
                break

            # Stall detection
            progress = (m5_pos - prev_pos) * direction
            if progress <= GRIP_POSITION_STALL:
                stall_reads += 1
            else:
                stall_reads = 0
            prev_pos = m5_pos

            if stall_reads >= 2 and elapsed > 0.15:
                print(
                    f"  Step {old_target} → {current_target}: "
                    f"pos={m5_pos}, load={m5_load}, current={m5_current} mA "
                    f"← STALL DETECTED"
                )
                contact_detected = True
                contact_position = m5_pos
                contact_reason = "stall"
                incremental_outcome = "contact"
                break

            # Don't hang on single small step
            if time.time() - step_start >= max(0.15, GRIP_POLL_INTERVAL * 3):
                break

        if contact_detected or incremental_outcome == "timeout":
            break
    else:
        # The while-loop condition became False ⇒ the claw reached the
        # closed limit without detecting any contact.
        incremental_outcome = "reached_limit"

    # 5. Secure close — ALWAYS command CLAW_CLOSED_POS regardless of
    #    whether the incremental loop found contact, timed out, or
    #    reached the limit.  The motor needs time to actually reach the
    #    target (or stall against a ball) before we read feedback.
    print(f"\n[SECURE CLOSE] Commanding CLAW_CLOSED_POS={CLAW_CLOSED_POS} "
          f"(incremental outcome: {incremental_outcome})…")
    goal["m5"] = CLAW_CLOSED_POS
    send_positions(goal)
    print(f"  Waiting {_SECURE_CLOSE_SETTLE:.1f}s for motor to reach target or stall…")
    time.sleep(_SECURE_CLOSE_SETTLE)

    # 6. Read final feedback AFTER the secure close settle time
    final = read_claw_feedback()
    final_pos = final.get("position") or CLAW_CLOSED_POS
    final_load = final.get("load") or 0
    final_current = final.get("current_ma") or 0
    blocked_steps = _claw_blocked_from_closed(final_pos)

    print(
        f"  Final feedback (after secure close): pos={final_pos}, load={final_load}, "
        f"current={final_current} mA, blocked={blocked_steps}"
    )

    # 7. Restore defaults
    try:
        send_raw({"cmd": "set_current_limit", "id": 5, "value": M5_DEFAULT_CURRENT_LIMIT})
    except Exception:
        pass
    try:
        send_raw({"cmd": "set_profile", "vel": CAL_PROFILE_VEL, "acc": CAL_PROFILE_ACC})
    except Exception:
        pass

    return {
        "contact_detected": contact_detected,
        "contact_position": contact_position,
        "contact_reason": contact_reason,
        "incremental_outcome": incremental_outcome,
        "final_position": final_pos,
        "final_load": final_load,
        "final_current_ma": final_current,
        "blocked_steps": blocked_steps,
        "grip_confirmed": feedback_confirms_grip(final),
    }


def print_analysis(result: dict):
    """Print the detailed grip analysis with thresholds and verdicts."""
    pos = result["final_position"]
    load = result["final_load"]
    current = result["final_current_ma"]
    blocked = result["blocked_steps"]
    current_threshold = _grip_current_contact_threshold()

    # Position checks
    minimally_blocked = blocked >= GRIP_MIN_BLOCKED_WITH_SENSOR
    strongly_blocked = blocked > GRIP_MIN_BALL_BLOCKED_STEPS
    load_ok = load is not None and load >= GRIP_LOAD_THRESHOLD
    current_ok = current is not None and current >= current_threshold
    grip = result["grip_confirmed"]

    def _check(ok: bool, label: str, detail: str) -> str:
        icon = "✅" if ok else "❌"
        tag = "YES" if ok else "NO"
        return f"  {label}: {icon} {tag} ({detail})"

    # Zero-load / zero-current flag (strongest no-ball indicator)
    no_resistance = (load is not None and load <= 0) and (current is not None and current < 5)

    print(f"\n{'═' * 40}")
    print(f"  GRIP ANALYSIS")
    print(f"{'═' * 40}")
    print(f"  Incremental outcome: {result.get('incremental_outcome', 'N/A')}")
    print(f"  Claw position:    {pos}")
    print(f"  CLAW_CLOSED_POS:  {CLAW_CLOSED_POS}")
    print(f"  Blocked steps:    {blocked}")
    print(f"  Load:             {load}")
    print(f"  Current:          {current} mA")
    if no_resistance:
        print(f"  ⚠️  Zero load + near-zero current → NO RESISTANCE (no ball)")
    if result["contact_detected"]:
        print(f"  Contact at:       {result['contact_position']} (reason: {result['contact_reason']})")

    print(f"\n  Thresholds:")
    print(f"    GRIP_MIN_BLOCKED_WITH_SENSOR: {GRIP_MIN_BLOCKED_WITH_SENSOR} (need ≥{GRIP_MIN_BLOCKED_WITH_SENSOR} with sensor confirm)")
    print(f"    GRIP_MIN_BALL_BLOCKED_STEPS:  {GRIP_MIN_BALL_BLOCKED_STEPS} (need >{GRIP_MIN_BALL_BLOCKED_STEPS} for position-only)")
    print(f"    GRIP_LOAD_THRESHOLD:          {GRIP_LOAD_THRESHOLD}")
    print(f"    GRIP_LOAD_DETECT:             {GRIP_LOAD_DETECT} (contact detection)")
    print(f"    Current contact threshold:    {current_threshold} mA (40% of {GRIP_CURRENT_LIMIT})")

    print()
    print(_check(minimally_blocked, f"Min position (blocked ≥ {GRIP_MIN_BLOCKED_WITH_SENSOR:>3})", f"{blocked} steps"))
    print(_check(strongly_blocked, f"Strong pos   (blocked > {GRIP_MIN_BALL_BLOCKED_STEPS:>3})", f"{blocked} steps"))
    print(_check(load_ok, f"Load         (≥ {GRIP_LOAD_THRESHOLD:>3})          ", f"{load}"))
    print(_check(current_ok, f"Current      (≥ {current_threshold:>3} mA)       ", f"{current} mA"))

    print()
    if grip:
        print("  ╔═══════════════════════════╗")
        print("  ║   ✅  GRIP DETECTED       ║")
        print("  ╚═══════════════════════════╝")
    else:
        print("  ╔═══════════════════════════╗")
        print("  ║   ❌  NO GRIP DETECTED    ║")
        print("  ╚═══════════════════════════╝")


# ── Main ─────────────────────────────────────────────────────────────

def main():
    # Parse optional port argument
    port = sys.argv[1] if len(sys.argv) > 1 else None

    print("═══════════════════════════════════════")
    print("  CLAW GRIP DIAGNOSTIC TOOL")
    print("═══════════════════════════════════════")
    print()

    try:
        _get_serial(port)
    except Exception as e:
        print(f"[ERROR] Could not open serial port: {e}")
        print("  Usage: python 11_claw_grip_test.py [/dev/cu.usbmodemXXX]")
        sys.exit(1)

    try:
        # ── Move to neutral ──────────────────────────────────────────
        print("[INFO] Moving arm to neutral position…")
        goto(NEUTRAL, pause_s=3.0)

        # Use IK to reach a comfortable observation pose
        try:
            from ik.solver import ArmIK

            arm = ArmIK()
            safe_pos = arm.solve(20, 0, 15)
            print("[INFO] Moving to observation pose (20, 0, 15)…")
            goto(safe_pos, pause_s=2.5)
        except Exception as e:
            print(f"[WARN] IK solve failed ({e}), staying at neutral")
            safe_pos = NEUTRAL.copy()

        # ── Grip test loop ───────────────────────────────────────────
        while True:
            # Open the claw
            print(f"\n[INFO] Opening claw (pos={CLAW_OPEN_POS})…")
            goal = safe_pos.copy()
            goal["m5"] = CLAW_OPEN_POS
            goto(goal, pause_s=1.0)

            # Wait for user
            print()
            input("Place a ball in the claw and press Enter "
                  "(or press Enter without a ball to test empty close)… ")

            # Run grip test
            result = run_grip_test(safe_pos)

            # Print analysis
            print_analysis(result)

            # Ask to repeat
            print()
            again = input("Test again? (y/n): ").strip().lower()
            if again not in ("y", "yes"):
                break

        # ── Cleanup ──────────────────────────────────────────────────
        print("\n[INFO] Opening claw for cleanup…")
        cleanup = safe_pos.copy()
        cleanup["m5"] = CLAW_OPEN_POS
        goto(cleanup, pause_s=1.0)

        print("[INFO] Returning to neutral…")
        goto(NEUTRAL, pause_s=2.0)

        print("[INFO] Disabling torque…")
        send_raw({"cmd": "torque_off"})

    except KeyboardInterrupt:
        print("\n\n[INFO] Interrupted by user.")
    except Exception:
        traceback.print_exc()
    finally:
        # Safety: try to open claw and disable torque
        try:
            ser = _get_serial()
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            send_raw({"cmd": "torque_off"})
        except Exception:
            pass

    print("\n═══════════════════════════════════════")
    print("  DONE.")
    print("═══════════════════════════════════════")


if __name__ == "__main__":
    main()
