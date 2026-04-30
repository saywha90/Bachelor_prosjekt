#!/usr/bin/env python3
"""
test_pick.py
============
End-to-end pick-and-place verification — Step 8.

Runs the full pipeline (detect → approach → grab → lift → bin)
at 5 positions across the workspace.  Scores each pick attempt
and prints a diagnostic summary.  Works with or without a camera
(manual coordinate entry fallback).

Saves results to pick_test_results.json.

Usage:
    python test_pick.py

Author: Bachelor Project 2026 – Autonomia
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import time
import math
from pathlib import Path

# ── Make sibling packages importable from src/ik/ ────────────────────
from ik.solver import ArmIK
from config.arm import (HOME_POSITION, BINS,
                        GRAB_HEIGHT, APPROACH_HEIGHT, CLEARANCE_HEIGHT,
                        GRIP_CURRENT_LIMIT, M5_DEFAULT_CURRENT_LIMIT)

RED_BIN    = BINS["RED_BIN"]
BLUE_BIN   = BINS["BLUE_BIN"]
REJECT_BIN = BINS["REJECT_BIN"]

# ── Claw constants ───────────────────────────────────────────────────
CLAW_OPEN_POS = 2016
CLAW_CLOSED_POS = 2890

# ── Timing ───────────────────────────────────────────────────────────
MOVE_SETTLE = 1.5

# ── Serial wrapper ───────────────────────────────────────────────────
# Identical lazy-singleton pattern used by calibrate_joints.py,
# calibrate_sag.py, and calibrate_claw.py.

SERIAL_PORT = "/dev/cu.usbmodem101"
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

    # Set a conservative motion profile
    _ser.write((json.dumps({"cmd": "set_profile", "vel": 80, "acc": 20}) + "\n").encode())
    time.sleep(0.3)
    _ser.readline()
    print("[SERIAL] Ready (profile: vel=80, acc=20)")
    return _ser


# ── Helpers ──────────────────────────────────────────────────────────

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


def goto(positions: dict, pause_s: float = 1.5):
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


def send_claw(pos):
    """Move only the claw (m5) while keeping other motors in place."""
    current = read_positions()
    if current is None:
        print("  ⚠️  Cannot read positions — sending claw command with defaults")
        send_command({"m5": pos})
        time.sleep(0.8)
        return
    current["m5"] = pos
    send_command(current)
    time.sleep(0.8)


# ── Test positions across the workspace ──────────────────────────────
TEST_POSITIONS = [
    {"name": "Centre", "x": 20, "y": 0},
    {"name": "Near",   "x": 14, "y": 0},
    {"name": "Far",    "x": 32, "y": 0},
    {"name": "Left",   "x": 20, "y": -12},
    {"name": "Right",  "x": 20, "y": 12},
]


# ══════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════

def main():
    # ── Banner ───────────────────────────────────────────────────────
    print("╔═══════════════════════════════════════════════════════════╗")
    print("║         END-TO-END PICK TEST — Step 8                    ║")
    print("╠═══════════════════════════════════════════════════════════╣")
    print("║  Tests the full pipeline: detect → move → grab → bin     ║")
    print("║                                                          ║")
    print("║  You'll need: coloured balls (red/blue), camera + arm    ║")
    print("╚═══════════════════════════════════════════════════════════╝")
    print()
    input("Clear the workspace around the arm, then press ENTER to start... ")

    # ── Initialise systems ───────────────────────────────────────────
    arm = ArmIK()
    _get_serial()  # connect arm

    # Try to start vision
    has_vision = False
    vision = None
    try:
        from ik.vision_bridge import VisionBridge
        vision = VisionBridge()
        vision.start()
        has_vision = True
        print("  ✅ Camera connected — using automated detection.\n")
    except Exception as e:
        print(f"  ⚠️  Camera not available: {e}")
        print("  Running in MANUAL mode — you'll enter ball positions by hand.\n")

    # ── Move to HOME ─────────────────────────────────────────────────
    print("Moving to HOME position…")
    home_sol = arm.solve(*HOME_POSITION)
    goto(home_sol, pause_s=2.0)
    send_claw(CLAW_OPEN_POS)

    # ── Run test loop ────────────────────────────────────────────────
    results = []

    for i, pos in enumerate(TEST_POSITIONS):
        print(f"\n{'=' * 50}")
        print(f"  Test {i + 1}/{len(TEST_POSITIONS)}: {pos['name']} "
              f"(X={pos['x']}, Y={pos['y']})")
        print(f"{'=' * 50}")

        # Return to home + open claw
        goto(arm.solve(*HOME_POSITION), pause_s=1.5)
        send_claw(CLAW_OPEN_POS)

        input(f"  Place a ball at approximately X={pos['x']}, "
              f"Y={pos['y']} cm. Press ENTER...")

        # ── DETECT ───────────────────────────────────────────────────
        detected_x, detected_y, color = None, None, "unknown"
        offset = 0.0

        if has_vision:
            print("  Scanning for ball...")
            for attempt in range(6):
                detections = vision.scan_for_balls()
                if detections:
                    # Find closest to expected position
                    best = min(
                        detections,
                        key=lambda d: math.sqrt(
                            (d["x"] - pos["x"]) ** 2
                            + (d["y"] - pos["y"]) ** 2
                        ),
                    )
                    detected_x, detected_y = best["x"], best["y"]
                    color = best.get("color", "unknown")
                    break
                time.sleep(0.5)

            if detected_x is not None:
                offset = math.sqrt(
                    (detected_x - pos["x"]) ** 2
                    + (detected_y - pos["y"]) ** 2
                )
                print(f"  Detected: ({detected_x:.1f}, {detected_y:.1f}) cm, "
                      f"color={color}")
                print(f"  Offset from expected: {offset:.1f} cm")
            else:
                print("  ❌ Ball not detected!")
                manual = input(
                    "  Enter detected position manually (x,y) or 'skip': "
                ).strip()
                if manual.lower() == "skip":
                    results.append({
                        "name": pos["name"],
                        "status": "skip",
                        "reason": "not detected",
                    })
                    continue
                detected_x, detected_y = [float(v) for v in manual.split(",")]
        else:
            manual = input(
                f"  Enter ball position (x,y) or press ENTER to use "
                f"({pos['x']},{pos['y']}): "
            ).strip()
            if manual:
                detected_x, detected_y = [float(v) for v in manual.split(",")]
            else:
                detected_x, detected_y = pos["x"], pos["y"]

        # ── APPROACH ─────────────────────────────────────────────────
        # NOTE: This manual test intentionally uses a 2-step descent
        # (approach height → grab height) for operator safety during
        # calibration.  The production code in main.py uses a single
        # direct move to GRAB_HEIGHT (see ADR-003).
        print("  Moving to approach height...")
        try:
            approach = arm.solve(detected_x, detected_y, APPROACH_HEIGHT)
            goto(approach)
        except Exception as e:
            print(f"  ❌ IK failed at approach: {e}")
            results.append({
                "name": pos["name"],
                "status": "fail",
                "reason": f"IK approach: {e}",
            })
            continue

        # ── LOWER TO GRAB ────────────────────────────────────────────
        print("  Lowering to grab height...")
        try:
            grab = arm.solve(detected_x, detected_y, GRAB_HEIGHT)
            goto(grab)
        except Exception as e:
            print(f"  ❌ IK failed at grab height: {e}")
            results.append({
                "name": pos["name"],
                "status": "fail",
                "reason": f"IK grab: {e}",
            })
            continue

        # ── CLOSE CLAW (with current limit safety) ───────────────────
        print(f"  Setting M5 current limit to {GRIP_CURRENT_LIMIT} mA for safe grip...")
        ser = _get_serial()
        ser.write((json.dumps({"cmd": "set_current_limit", "id": 5, "value": GRIP_CURRENT_LIMIT}) + "\n").encode())
        ser.readline()  # consume response

        print("  Closing claw...")
        send_claw(CLAW_CLOSED_POS)
        time.sleep(0.5)

        # Restore default current limit
        print(f"  Restoring M5 current limit to {M5_DEFAULT_CURRENT_LIMIT} mA")
        ser.write((json.dumps({"cmd": "set_current_limit", "id": 5, "value": M5_DEFAULT_CURRENT_LIMIT}) + "\n").encode())
        ser.readline()  # consume response

        # ── LIFT ─────────────────────────────────────────────────────
        print("  Lifting...")
        try:
            lift = arm.solve(detected_x, detected_y, CLEARANCE_HEIGHT)
            goto(lift)
        except Exception as e:
            print(f"  ⚠️  IK failed at clearance height: {e}")

        # ── ASK USER ─────────────────────────────────────────────────
        result = input(
            "  Did the arm grab the ball cleanly? (y/n/partial): "
        ).strip().lower()

        if result == "y":
            # Move to bin
            if color == "red":
                bin_pos = RED_BIN
                bin_name = "red"
            elif color == "blue":
                bin_pos = BLUE_BIN
                bin_name = "blue"
            else:
                bin_pos = REJECT_BIN
                bin_name = "reject"

            print(f"  Moving to {bin_name} bin...")
            try:
                bin_sol = arm.solve(*bin_pos)
                goto(bin_sol)
                send_claw(CLAW_OPEN_POS)
                time.sleep(0.5)
            except Exception as e:
                print(f"  ⚠️  Bin move failed: {e} — dropping here")
                send_claw(CLAW_OPEN_POS)

            results.append({
                "name": pos["name"],
                "status": "pass",
                "offset": offset if has_vision else 0,
            })
        elif result == "partial":
            send_claw(CLAW_OPEN_POS)
            reason = input(
                "  What went wrong? (e.g., 'claw too high', "
                "'offset left 1cm'): "
            ).strip()
            results.append({
                "name": pos["name"],
                "status": "partial",
                "reason": reason,
            })
        else:
            send_claw(CLAW_OPEN_POS)
            reason = input("  What went wrong? ").strip()
            results.append({
                "name": pos["name"],
                "status": "fail",
                "reason": reason,
            })

    # ── Results summary ──────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  END-TO-END PICK TEST RESULTS")
    print("=" * 60)

    passes = sum(1 for r in results if r["status"] == "pass")
    total = len(results)

    for r in results:
        if r["status"] == "pass":
            icon = "✅"
        elif r["status"] == "partial":
            icon = "⚠️"
        elif r["status"] == "fail":
            icon = "❌"
        else:
            icon = "⏭️"

        line = f"  {icon} {r['name']}: {r['status']}"
        if "offset" in r:
            line += f" (detection offset: {r['offset']:.1f} cm)"
        if "reason" in r:
            line += f" — {r['reason']}"
        print(line)

    pct = (passes / total * 100) if total > 0 else 0
    print(f"\n  Score: {passes}/{total} ({pct:.0f}%)")

    if pct >= 80:
        print("  ✅ PASS — Calibration is good! System ready for operation.")
    elif pct >= 50:
        print("  ⚠️  MARGINAL — Some adjustments needed. "
              "See diagnostic table below.")
    else:
        print("  ❌ FAIL — Significant calibration issues. "
              "Review the diagnostic table.")

    # ── Diagnostic hints ─────────────────────────────────────────────
    print("\n  DIAGNOSTIC HINTS:")
    print("  ┌───────────────────────────┬──────────────────────────────┐")
    print("  │ Symptom                   │ Fix                          │")
    print("  ├───────────────────────────┼──────────────────────────────┤")
    print("  │ Consistent X/Y offset     │ Adjust CAMERA_OFFSET in     │")
    print("  │                           │ src/config/arm.py, or redo 7   │")
    print("  │ Variable offset           │ Redo homography (Step 6)    │")
    print("  │ Claw too high/low         │ Redo sag cal (Step 3)       │")
    print("  │ Ball squirts out          │ Redo claw cal (Step 2b)     │")
    print("  │ Wrong bin                 │ Redo HSV tuning (Step 4-5)  │")
    print("  └───────────────────────────┴──────────────────────────────┘")

    # ── Save results to JSON ─────────────────────────────────────────
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "results": results,
        "score": f"{passes}/{total}",
        "pass_rate": pct,
    }
    out_path = Path(__file__).resolve().parent / "pick_test_results.json"
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\n  Results saved to {out_path}")

    # ── Return to HOME ───────────────────────────────────────────────
    print("\nReturning to HOME position…")
    goto(arm.solve(*HOME_POSITION), pause_s=2.0)
    send_claw(CLAW_OPEN_POS)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n  ⛔ Interrupted — returning to HOME…")
        try:
            arm = ArmIK()
            home_sol = arm.solve(*HOME_POSITION)
            goto(home_sol, pause_s=2.0)
            send_claw(CLAW_OPEN_POS)
        except Exception:
            pass
        print("  Done. Exiting.")
        sys.exit(1)
    except Exception as e:
        print(f"\n  ⚠️  Unexpected error: {e}")
        print("  Attempting to return to HOME…")
        try:
            arm = ArmIK()
            home_sol = arm.solve(*HOME_POSITION)
            goto(home_sol, pause_s=2.0)
            send_claw(CLAW_OPEN_POS)
        except Exception:
            pass
        sys.exit(1)
