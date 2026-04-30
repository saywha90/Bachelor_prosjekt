#!/usr/bin/env python3
"""
calibrate_sag.py
================
Sag (droop) calibration for the robotic arm — Step 3.

Measures gravity-induced droop at multiple horizontal reach distances
with sag compensation disabled.  Fits both linear and quadratic
compensation models to the measured errors and saves the coefficients
to sag_calibration.json (auto-loaded by ArmIK on startup).

Each reach point is measured 3 times and averaged to reduce ruler noise.

Usage:
    python calibrate_sag.py          # default test height = 2 cm (GRAB_HEIGHT)
    python calibrate_sag.py 8        # custom test height = 8 cm

Author: Bachelor Project 2026 – Autonomia
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import math
import time
from pathlib import Path

import numpy as np

from ik.solver import ArmIK

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
NEUTRAL = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911, "m5": 2048}

# Default test parameters — TEST_Z raised to 10 cm to avoid desk hits
# at long reaches (previously 2.0 cm, which caused the arm to hit the
# desk at reaches >= 24 cm, corrupting calibration data).
TEST_Z = 10.0  # cm — high enough to avoid desk contact at all reaches
TEST_REACHES = [6, 12, 18, 24, 30, 36]  # cm, along X axis (Y = 0); 6 cm for near-range coverage
NUM_REPEATS = 3  # measurements per reach point (averaged to reduce ruler noise)

# Desk-hit detection threshold (cm).  If measured_z is at or below this
# value the arm likely hit the desk surface and the reading is censored.
DESK_HIT_THRESHOLD = 0.3


# ── Measurement helpers ─────────────────────────────────────────────

def read_measurement(reach: float, trial: int = 1, total: int = 1) -> float:
    """Prompt the user for the measured claw-tip height, with input validation."""
    while True:
        if total > 1:
            raw = input(f"  Measurement {trial}/{total} — enter claw tip height in cm (reach={reach}): ").strip()
        else:
            raw = input(f"  Enter measured claw tip height in cm (reach={reach}): ").strip()
        try:
            value = float(raw)
            return value
        except ValueError:
            print("  ⚠️  Invalid input — please enter a number (e.g. 4.3)")


def read_averaged_measurement(reach: float, num_repeats: int = NUM_REPEATS) -> float:
    """Take multiple measurements at one reach point and return the average.

    Prompts the user *num_repeats* times, prints individual readings and
    the averaged result so the operator can spot outliers.
    """
    readings: list[float] = []
    for i in range(num_repeats):
        val = read_measurement(reach, trial=i + 1, total=num_repeats)
        readings.append(val)
    avg = sum(readings) / len(readings)
    spread = max(readings) - min(readings)
    print(f"  → Readings: {readings}  |  avg = {avg:.2f} cm  |  spread = {spread:.2f} cm")
    if spread > 0.5:
        print("  ⚠️  Spread > 0.5 cm — consider re-measuring this point.")
    return avg


def collect_data(arm: ArmIK, test_z: float, reaches: list,
                 num_repeats: int = NUM_REPEATS) -> list:
    """Move the arm to each test reach and collect user measurements.

    At each reach, the user measures *num_repeats* times; the average is
    recorded.  Returns a list of (reach, commanded_z, measured_z) tuples.
    """
    data = []
    for x in reaches:
        print(f"\n{'─'*50}")
        print(f"  Moving claw to X={x} cm, Y=0 cm, Z={test_z} cm")
        print(f"  (sag compensation OFF — all offsets zeroed)")
        print(f"  You will measure {num_repeats} time(s) per reach point.")
        print(f"{'─'*50}")

        try:
            solution = arm.solve(x, 0, test_z)
        except ValueError as e:
            print(f"  ⚠️  IK solve failed for reach={x} cm: {e}")
            print(f"  Skipping this reach distance.")
            continue

        goto(solution, pause_s=2.5)

        measured_z = read_averaged_measurement(x, num_repeats)
        data.append((float(x), test_z, measured_z))
        print(f"  ✓ Recorded: reach={x}, commanded_z={test_z}, measured_z={measured_z:.2f}")

    return data


# ── Analysis ────────────────────────────────────────────────────────

def fit_models(data: list):
    """Fit linear and quadratic compensation models to the measurement data.

    ALL data points — including desk-hit points — are used for fitting.
    Desk-hit points (measured_z <= DESK_HIT_THRESHOLD) are detected and
    warned about, but they provide valuable information about the error
    curve at longer reaches and are kept in the fit to prevent dangerous
    over-extrapolation from a linear-only model.

    Returns (linear_coeffs, quad_coeffs, linear_rmse, quad_rmse, desk_hit_indices).
    """
    # ── Detect desk-hit points (for warnings / annotation only) ─────
    desk_hit_indices: list[int] = []
    for i, (r, cmd_z, meas_z) in enumerate(data):
        if meas_z <= DESK_HIT_THRESHOLD:
            desk_hit_indices.append(i)
            print(f"  ⚠ Desk hit at reach={r}cm: measured_z={meas_z}cm "
                  f"(included in fit — error assumed ≈ +{cmd_z:.1f}cm)")

    # If ALL points are desk-hits, we can't get meaningful variation
    if len(desk_hit_indices) == len(data):
        print("\n  ❌ ERROR: ALL data points are desk hits.")
        print("     Cannot fit a meaningful model — every point is censored.")
        print("     Suggestion: re-run calibration with a higher TEST_Z "
              f"(current: {TEST_Z} cm).")
        raise ValueError("All data points are desk hits — "
                         "re-run with higher TEST_Z")

    if len(data) < 2:
        raise ValueError("Too few data points for fitting "
                         f"(need ≥2, got {len(data)})")

    # Use ALL data points for fitting
    reaches = np.array([r for r, _, _ in data])
    errors = np.array([cmd_z - meas_z for _, cmd_z, meas_z in data])

    # Linear:   error = a * reach + b
    linear_coeffs = np.polyfit(reaches, errors, 1)

    # Quadratic: error = a * reach^2 + b * reach + c
    if len(data) >= 3:
        quad_coeffs = np.polyfit(reaches, errors, 2)
    else:
        # With <3 points quadratic is under-determined; mirror the linear fit
        quad_coeffs = np.array([0.0, linear_coeffs[0], linear_coeffs[1]])

    # RMSE for each fit (computed on ALL points)
    linear_rmse = float(np.sqrt(np.mean(
        (np.polyval(linear_coeffs, reaches) - errors) ** 2)))
    quad_rmse = float(np.sqrt(np.mean(
        (np.polyval(quad_coeffs, reaches) - errors) ** 2)))

    return linear_coeffs, quad_coeffs, linear_rmse, quad_rmse, desk_hit_indices


def print_results(data, linear_coeffs, quad_coeffs, linear_rmse, quad_rmse,
                  desk_hit_indices=None):
    """Print a formatted results table and model comparison.

    Desk-hit rows are annotated with "(desk hit)" but ARE included in
    the fit — they provide valuable information about the error curve.
    """
    if desk_hit_indices is None:
        desk_hit_indices = []

    all_reaches = np.array([r for r, _, _ in data])
    linear_pred = np.polyval(linear_coeffs, all_reaches)
    quad_pred = np.polyval(quad_coeffs, all_reaches)

    print()
    print("┌────────────────────────────────────────────────────────────────────────────┐")
    print("│                        SAG CALIBRATION RESULTS                             │")
    print("├─────────┬──────────┬───────────────────┬──────────┬─────────────┬──────────┤")
    print("│ Reach   │ Cmd Z    │ Meas Z            │ Error    │ Lin. Pred   │ Qd. Pred │")
    print("├─────────┼──────────┼───────────────────┼──────────┼─────────────┼──────────┤")

    for i, (reach, cmd_z, meas_z) in enumerate(data):
        err = cmd_z - meas_z
        lp = linear_pred[i]
        qp = quad_pred[i]
        if i in desk_hit_indices:
            meas_str = f"{meas_z:5.1f} (desk hit)"
        else:
            meas_str = f"{meas_z:5.1f} cm        "
        print(f"│ {reach:5.1f} cm │ {cmd_z:6.1f} cm │ {meas_str} │ {err:6.2f} cm │ {lp:7.2f} cm   │ {qp:6.2f} cm │")

    print("└─────────┴──────────┴───────────────────┴──────────┴─────────────┴──────────┘")

    if desk_hit_indices:
        print(f"\n  ℹ {len(desk_hit_indices)} desk-hit point(s) detected (included in fit).")

    print()
    print(f"Linear fit:  slope = {linear_coeffs[0]:.4f}, intercept = {linear_coeffs[1]:.4f}  "
          f"(RMSE: {linear_rmse:.3f} cm)")
    print(f"Quadratic:   a={quad_coeffs[0]:.6f}, b={quad_coeffs[1]:.4f}, "
          f"c={quad_coeffs[2]:.4f}   (RMSE: {quad_rmse:.3f} cm)")

    # Recommendation
    if quad_rmse < linear_rmse * 0.8:
        improvement = (1.0 - quad_rmse / linear_rmse) * 100.0
        print(f"\nRecommendation: Use quadratic model "
              f"(RMSE improved by {improvement:.0f}%)")
        print(f"  z_offset_quadratic (a) = {quad_coeffs[0]:.6f}")
        print(f"  z_offset_multiplier (b) = {quad_coeffs[1]:.4f}")
        print(f"  z_offset_constant (c)   = {quad_coeffs[2]:.4f}")
        recommended = "quadratic"
    else:
        print(f"\nRecommendation: Use linear model "
              f"(quadratic does not improve fit by ≥20%)")
        print(f"  slope (z_offset_multiplier) = {linear_coeffs[0]:.4f}")
        print(f"  intercept (z_offset_constant) = {linear_coeffs[1]:.4f}")
        recommended = "linear"

    return recommended


def save_calibration(data, test_z, linear_coeffs, quad_coeffs,
                     linear_rmse, quad_rmse, recommended,
                     desk_hit_indices=None):
    """Save calibration results to sag_calibration.json.

    ALL measurements (including desk-hit points) go in "measurements".
    The "desk_hits" field is a simple list of reach values where desk
    contact was detected — kept for informational purposes only.
    """
    if desk_hit_indices is None:
        desk_hit_indices = []

    all_measurements = []
    desk_hit_reaches = []
    for i, (r, cz, mz) in enumerate(data):
        all_measurements.append({"reach_cm": r, "commanded_z_cm": cz, "measured_z_cm": mz})
        if i in desk_hit_indices:
            desk_hit_reaches.append(r)

    calibration = {
        "test_z_cm": test_z,
        "measurements": all_measurements,
        "desk_hits": desk_hit_reaches,
        "linear": {
            "slope": float(linear_coeffs[0]),
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

    out_path = Path(__file__).resolve().parent.parent / "ik" / "sag_calibration.json"
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

    # Initialise IK with ALL sag parameters zeroed so residual calibration
    # does not influence the raw measurements.
    arm = ArmIK(
        z_offset_multiplier=0.0,
        z_offset_quadratic=0.0,
        z_offset_constant=0.0,
    )

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
        linear_coeffs, quad_coeffs, linear_rmse, quad_rmse, desk_hit_indices = fit_models(data)

        # ── Print results ──────────────────────────────────────────
        recommended = print_results(data, linear_coeffs, quad_coeffs,
                                    linear_rmse, quad_rmse, desk_hit_indices)

        # ── Save to JSON ───────────────────────────────────────────
        save_calibration(data, test_z, linear_coeffs, quad_coeffs,
                         linear_rmse, quad_rmse, recommended, desk_hit_indices)

        # ── Print next steps ───────────────────────────────────────
        print()
        print("=" * 60)
        print("NEXT STEPS")
        print("=" * 60)
        print()
        print(f"Recommended model: {recommended}")
        print()
        print(f"To apply the linear model, edit src/ik/solver.py:")
        print(f"    z_offset_multiplier: float = {linear_coeffs[0]:.4f}")
        print(f"    z_offset_constant:   float = {linear_coeffs[1]:.4f}")
        print()
        print(f"To apply the quadratic model, edit src/ik/solver.py:")
        print(f"    z_offset_quadratic:  float = {quad_coeffs[0]:.6f}")
        print(f"    z_offset_multiplier: float = {quad_coeffs[1]:.4f}")
        print(f"    z_offset_constant:   float = {quad_coeffs[2]:.4f}")
        print()
        print(f"NOTE: ArmIK auto-loads sag_calibration.json on startup.")
        print(f"      The JSON file has been saved — no manual edits needed.")
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
