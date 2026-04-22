#!/usr/bin/env python3
"""
calibrate_sag.py
================
Sag (droop) calibration for the robotic arm — Step 3.

Measures gravity-induced droop at multiple horizontal reach distances
with sag compensation disabled.  Fits both linear and quadratic
compensation models to the measured errors and saves the coefficients
to sag_calibration.json (auto-loaded by ArmIK on startup).

Usage:
    python calibrate_sag.py          # default test height = 5 cm
    python calibrate_sag.py 8        # custom test height = 8 cm

Author: Bachelor Project 2026 – Autonomia
"""

import json
import math
import sys
import time

import numpy as np

# Allow importing pi_kinematics from the same directory
sys.path.insert(0, ".")
from pi_kinematics import ArmIK

# ── Serial wrapper ──────────────────────────────────────────────────
# Identical lazy-singleton pattern used by calibrate_joints.py.
# Change SERIAL_PORT / SERIAL_BAUD to match your setup.

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


def prompt(msg: str):
    input(f"\n>>> {msg}\n    Press ENTER to continue...")


# ── Constants ───────────────────────────────────────────────────────
NEUTRAL = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 2048, "m5": 2048}

# Default test parameters
TEST_Z = 5.0  # cm above the desk — high enough to avoid hitting even with sag
TEST_REACHES = [12, 18, 24, 30, 36]  # cm, along X axis (Y = 0)


# ── Measurement helpers ─────────────────────────────────────────────

def read_measurement(reach: float) -> float:
    """Prompt the user for the measured claw-tip height, with input validation."""
    while True:
        raw = input(f"  Enter measured claw tip height in cm (reach={reach}): ").strip()
        try:
            value = float(raw)
            return value
        except ValueError:
            print("  ⚠️  Invalid input — please enter a number (e.g. 4.3)")


def collect_data(arm: ArmIK, test_z: float, reaches: list) -> list:
    """Move the arm to each test reach and collect user measurements.

    Returns a list of (reach, commanded_z, measured_z) tuples.
    """
    data = []
    for x in reaches:
        print(f"\n{'─'*50}")
        print(f"  Moving claw to X={x} cm, Y=0 cm, Z={test_z} cm")
        print(f"  (sag compensation OFF)")
        print(f"{'─'*50}")

        try:
            solution = arm.solve(x, 0, test_z)
        except ValueError as e:
            print(f"  ⚠️  IK solve failed for reach={x} cm: {e}")
            print(f"  Skipping this reach distance.")
            continue

        goto(solution, pause_s=2.5)

        measured_z = read_measurement(x)
        data.append((float(x), test_z, measured_z))
        print(f"  ✓ Recorded: reach={x}, commanded_z={test_z}, measured_z={measured_z}")

    return data


# ── Analysis ────────────────────────────────────────────────────────

def fit_models(data: list):
    """Fit linear and quadratic compensation models to the measurement data.

    Returns (linear_coeffs, quad_coeffs, linear_rmse, quad_rmse).
    """
    reaches = np.array([r for r, _, _ in data])
    errors = np.array([cmd_z - meas_z for _, cmd_z, meas_z in data])

    # Linear:   error = a * reach + b
    linear_coeffs = np.polyfit(reaches, errors, 1)

    # Quadratic: error = a * reach^2 + b * reach + c
    quad_coeffs = np.polyfit(reaches, errors, 2)

    # RMSE for each fit
    linear_rmse = float(np.sqrt(np.mean((np.polyval(linear_coeffs, reaches) - errors) ** 2)))
    quad_rmse = float(np.sqrt(np.mean((np.polyval(quad_coeffs, reaches) - errors) ** 2)))

    return linear_coeffs, quad_coeffs, linear_rmse, quad_rmse


def print_results(data, linear_coeffs, quad_coeffs, linear_rmse, quad_rmse):
    """Print a formatted results table and model comparison."""
    reaches = np.array([r for r, _, _ in data])
    errors = np.array([cmd_z - meas_z for _, cmd_z, meas_z in data])
    linear_pred = np.polyval(linear_coeffs, reaches)

    linear_multiplier = linear_coeffs[0]

    print()
    print("┌─────────────────────────────────────────────────────────┐")
    print("│                SAG CALIBRATION RESULTS                  │")
    print("├─────────┬──────────┬──────────┬──────────┬──────────────┤")
    print("│ Reach   │ Cmd Z    │ Meas Z   │ Error    │ Lin. Pred    │")
    print("├─────────┼──────────┼──────────┼──────────┼──────────────┤")

    for i, (reach, cmd_z, meas_z) in enumerate(data):
        err = cmd_z - meas_z
        lp = linear_pred[i]
        print(f"│ {reach:5.1f} cm │ {cmd_z:6.1f} cm │ {meas_z:6.1f} cm │ {err:6.2f} cm │ {lp:8.2f} cm   │")

    print("└─────────┴──────────┴──────────┴──────────┴──────────────┘")

    print()
    print(f"Linear fit:  z_offset_multiplier = {linear_multiplier:.4f}  "
          f"(RMSE: {linear_rmse:.3f} cm)")
    print(f"Quadratic:   a={quad_coeffs[0]:.6f}, b={quad_coeffs[1]:.4f}, "
          f"c={quad_coeffs[2]:.4f}   (RMSE: {quad_rmse:.3f} cm)")

    # Recommendation
    if quad_rmse < linear_rmse * 0.8:
        improvement = (1.0 - quad_rmse / linear_rmse) * 100.0
        print(f"\nRecommendation: Use quadratic model "
              f"(RMSE improved by {improvement:.0f}%)")
        recommended = "quadratic"
    else:
        print(f"\nRecommendation: Use linear model "
              f"(quadratic does not improve fit by ≥20%)")
        recommended = "linear"

    return recommended


def save_calibration(data, test_z, linear_coeffs, quad_coeffs,
                     linear_rmse, quad_rmse, recommended):
    """Save calibration results to sag_calibration.json."""
    linear_multiplier = float(linear_coeffs[0])

    calibration = {
        "test_z_cm": test_z,
        "measurements": [
            {"reach": r, "commanded_z": cz, "measured_z": mz}
            for r, cz, mz in data
        ],
        "linear": {
            "z_offset_multiplier": linear_multiplier,
            "intercept": float(linear_coeffs[1]),
            "rmse_cm": float(linear_rmse),
        },
        "quadratic": {
            "a": float(quad_coeffs[0]),
            "b": float(quad_coeffs[1]),
            "c": float(quad_coeffs[2]),
            "rmse_cm": float(quad_rmse),
        },
        "recommended_model": recommended,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    out_path = "src/IK/sag_calibration.json"
    with open(out_path, "w") as f:
        json.dump(calibration, f, indent=2)
    print(f"\n✓ Calibration saved to {out_path}")

    return calibration


# ── Main ────────────────────────────────────────────────────────────

def main():
    # Parse optional test Z from command line
    test_z = TEST_Z
    if len(sys.argv) > 1:
        try:
            test_z = float(sys.argv[1])
            print(f"Using custom test Z = {test_z} cm")
        except ValueError:
            print(f"⚠️  Invalid CLI argument '{sys.argv[1]}', using default Z = {test_z} cm")

    print("=" * 60)
    print("ARM SAG / DROOP CALIBRATION")
    print("=" * 60)
    print()
    print(f"This script moves the claw to Z = {test_z} cm at several")
    print(f"horizontal reaches with sag compensation OFF, then asks you")
    print(f"to measure the actual claw-tip height.")
    print()
    print(f"Test reaches (cm): {TEST_REACHES}")
    print()
    input("Clear the workspace around the arm, then press ENTER to start... ")

    # Initialise IK with sag compensation DISABLED
    arm = ArmIK(z_offset_multiplier=0.0)

    try:
        # ── Move to neutral first ──────────────────────────────────
        print("\nMoving to NEUTRAL position…")
        goto(NEUTRAL, pause_s=3.0)

        # ── Collect measurements ───────────────────────────────────
        data = collect_data(arm, test_z, TEST_REACHES)

        if len(data) < 2:
            print("\n⚠️  Need at least 2 measurements to fit a model. Aborting.")
            goto(NEUTRAL, pause_s=2.0)
            return

        # ── Fit models ─────────────────────────────────────────────
        linear_coeffs, quad_coeffs, linear_rmse, quad_rmse = fit_models(data)

        # ── Print results ──────────────────────────────────────────
        recommended = print_results(data, linear_coeffs, quad_coeffs,
                                    linear_rmse, quad_rmse)

        # ── Save to JSON ───────────────────────────────────────────
        save_calibration(data, test_z, linear_coeffs, quad_coeffs,
                         linear_rmse, quad_rmse, recommended)

        # ── Print next steps ───────────────────────────────────────
        linear_multiplier = float(linear_coeffs[0])
        print()
        print("=" * 60)
        print("NEXT STEPS")
        print("=" * 60)
        print()
        print(f"To apply the linear model, edit pi_kinematics.py line 53:")
        print(f"    z_offset_multiplier: float = {linear_multiplier:.4f}")
        print()
        print(f"To apply the quadratic model, see README or update")
        print(f"pi_kinematics.py to load sag_calibration.json.")
        print()

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
    except Exception as e:
        print(f"\n⚠️  Error: {e}")
    finally:
        # ── Return to neutral ──────────────────────────────────────
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
