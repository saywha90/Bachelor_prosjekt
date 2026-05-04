"""
calibrate_joints.py
===================
Joint-by-joint calibration diagnostic.

Goal: figure out which motor has a wrong sign / zero / or which link
length is off, by commanding ONE motor at a time and asking you to
measure the actual claw tip position.

Run this INSTEAD of the main pipeline. It does not use IK. It just
drives one servo at a time and pauses for you to measure.

Usage:
    python calibrate_joints.py

Author: Bachelor Project 2026 – Autonomia 
"""


import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import json
import time

# ── Serial wrapper ──────────────────────────────────────────────────
# main.py opens a serial.Serial and sends JSON-encoded dicts directly.
# We replicate that pattern here with two thin helpers so the rest of
# the script stays clean.
#
# Change SERIAL_PORT / SERIAL_BAUD to match your setup (values taken
# from main.py defaults).

SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD = 115200

_ser = None  # lazily initialised on first call


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


# ── Helper ──────────────────────────────────────────────────────────
def goto(positions: dict, pause_s: float = 2.0):
    """Send absolute motor targets and wait for motion to finish."""
    send_command(positions)
    time.sleep(pause_s)


def prompt(msg: str):
    input(f"\n>>> {msg}\n    Press ENTER when measured... ")


# ── Known-pose reference ────────────────────────────────────────────
NEUTRAL = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911, "m5": 2048}


def test_1_neutral():
    print("\n" + "=" * 60)
    print("TEST 1 — Neutral pose (all motors = 2048)")
    print("=" * 60)
    print("Expected: shoulder points STRAIGHT UP, elbow STRAIGHT,")
    print("          wrist in line with forearm (claw points up).")
    goto(NEUTRAL, pause_s=3.0)
    prompt("Is the arm pointing straight up with all links aligned?\n"
           "    If NO — note which joint is off and by roughly how many degrees.")


def test_2_shoulder_only():
    print("\n" + "=" * 60)
    print("TEST 2 — Shoulder sweep (only m2 moves)")
    print("=" * 60)
    print("We'll move the shoulder +/- 500 steps from centre.")
    print("Everything else stays at 2048.")

    goto({**NEUTRAL, "m2": 2548}, pause_s=2.5)
    prompt("m2 = 2548 (+500). Which direction did the upper arm tilt?\n"
           "    (forward = away from base, or backward = toward base?)")

    goto({**NEUTRAL, "m2": 1548}, pause_s=2.5)
    prompt("m2 = 1548 (-500). Which direction now?")

    goto(NEUTRAL, pause_s=2.0)


def test_3_elbow_only():
    print("\n" + "=" * 60)
    print("TEST 3 — Elbow sweep (only m3 moves)")
    print("=" * 60)
    base = {**NEUTRAL, "m2": 2400}
    goto(base, pause_s=2.5)
    prompt("Shoulder tilted forward. Ready to test elbow?")

    goto({**base, "m3": 2548}, pause_s=2.5)
    prompt("m3 = 2548 (+500). Forearm bent up or down relative to upper arm?")

    goto({**base, "m3": 1548}, pause_s=2.5)
    prompt("m3 = 1548 (-500). Forearm bent which way now?")

    goto(NEUTRAL, pause_s=2.0)


def test_4_wrist_only():
    print("\n" + "=" * 60)
    print("TEST 4 — Wrist sweep (only m4 moves)")
    print("=" * 60)
    base = {**NEUTRAL, "m2": 2400, "m3": 1600}
    goto(base, pause_s=2.5)
    prompt("Arm posed so the forearm is roughly horizontal. Ready?")

    goto({**base, "m4": 2548}, pause_s=2.5)
    prompt("m4 = 2548 (+500). Claw tip pointing up or down relative to forearm?")

    goto({**base, "m4": 1548}, pause_s=2.5)
    prompt("m4 = 1548 (-500). Claw tip pointing which way now?")

    goto(NEUTRAL, pause_s=2.0)


def test_5_known_geometry():
    print("\n" + "=" * 60)
    print("TEST 5 — Known geometry check")
    print("=" * 60)
    print("Setting shoulder horizontal forward, elbow at 90° down,")
    print("wrist straight (claw continues forearm direction).")
    print()
    print("If the driver signs are correct, the upper arm should")
    print("point horizontal forward (+X), the forearm straight down,")
    print("and the claw continue straight down from the wrist.")
    print()
    print("Expected claw tip position (if L1=25.5, L2=23.0, L3=22.0,")
    print("shoulder_height=33.0):")
    print("    x = L1 = 25.5 cm")
    print("    y = 0")
    print("    z = shoulder_height - L2 - L3 = 33 - 23 - 22.0 = -12.0 cm")
    print("    (i.e. the claw tip would be 12.0 cm BELOW the desk;")
    print("     the arm will hit the desk first — that's fine, just")
    print("     measure the claw tip height above the desk when stopped.)")
    print()
    pose = {
        "m1": 2048,
        "m2": 2048 + 1024,
        "m3": 2048 - 1024,
        "m4": 1911,    # M4 mechanical centre (3D-printed mount offset)
        "m5": 2048,
    }
    print(f"Commanding: {pose}")
    goto(pose, pause_s=3.0)
    prompt("Measure:\n"
           "    (a) upper arm horizontal? (yes/no — if no, which way tilted?)\n"
           "    (b) forearm pointing straight down? (yes/no — if no, which way?)\n"
           "    (c) claw tip: how many cm forward of the shoulder axis (X)?\n"
           "    (d) claw tip: how many cm above the desk (Z)?")

    pose2 = {**pose, "m2": 2048 - 1024}
    print(f"\nNow trying opposite shoulder sign: {pose2}")
    goto(pose2, pause_s=3.0)
    prompt("Same questions — which of the two shoulder signs produced\n"
           "    a horizontal upper arm pointing forward (away from base)?")

    goto(NEUTRAL, pause_s=2.0)


def main():
    global _ser
    print("=" * 60)
    print("ARM JOINT CALIBRATION DIAGNOSTIC")
    print("=" * 60)
    print("Work through each test. Write the answers down — we'll use")
    print("them to fix the sign/zero errors in src/ik/solver.py.")
    print()
    input("Clear the workspace around the arm, then press ENTER to start... ")

    try:
        test_1_neutral()
        test_2_shoulder_only()
        test_3_elbow_only()
        test_4_wrist_only()
        test_5_known_geometry()

        print("\n" + "=" * 60)
        print("DONE. Paste your answers back in chat and I'll tell you")
        print("exactly which lines in src/ik/solver.py to change.")
        print("=" * 60)
    finally:
        if _ser is not None:
            _ser.close()
            print("Serial port closed.")


if __name__ == "__main__":
    main()
