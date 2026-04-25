"""
main.py
=======
Master sorting loop for the autonomous robotic arm.

Implements a state-machine that drives the arm through:
    HOME → MOVE_TO_SCAN_POSE → SCANNING → APPROACHING → GRABBING →
    SORTING (lift + move to bin) → DROPPING → MOVE_TO_SCAN_POSE →
    (rescan or next from queue)

Camera integration
------------------
Uses a wrist-mounted OAK-D S2 camera.  The arm parks at SCAN_POSE
(joint-space positions defined in config/arm.py) before every vision
scan, then approaches in a single full move.

When ``--real-camera`` is passed, uses ``VisionBridge`` to capture real
detections from the OAK-D camera and convert pixel positions to arm-frame
centimetres via a calibrated homography.

When ``--real-camera`` is omitted, the bridge returns canned fake detections
so the state machine and 3-D visualiser can be tested without hardware.

Uses 5 daisy-chained Dynamixel motors via ``mock_serial.MockSerial``
while hardware is unavailable.
Swap to ``serial.Serial`` once the OpenRB-150 is connected.

Author: Bachelor Project 2026 – Autonomia
"""

import argparse
import logging
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import json
import time
from enum import Enum, auto

import cv2

from ik.solver import ArmIK
from config.arm import (
    HOME_POSITION,
    GRAB_HEIGHT,
    APPROACH_HEIGHT,
    CLEARANCE_HEIGHT,
    GRAB_DWELL,
    RELEASE_DWELL,
    CAMERA_OFFSET_X,
    SCAN_POSE,
    STARTUP_PROFILE_VEL,
    STARTUP_PROFILE_ACC,
    SCAN_INTERVAL,
)
from simulation.mock_serial import MockSerial
from simulation.visualizer import ArmVisualizer
from ik.vision_bridge import VisionBridge

logger = logging.getLogger(__name__)

# ─── Serial / connection settings ─────────────────────────────────────
SERIAL_PORT = "/dev/cu.usbmodem2101"
SERIAL_BAUD     = 115200

# ─── Claw motor positions (Dynamixel steps) ───────────────────────────
CLAW_OPEN_POS   = 2048    # open/neutral position for gripper
CLAW_CLOSED_POS = 1600    # closed/grip position (tune on real hardware)

# ─── Movement settling time (seconds) ────────────────────────────────
#   After sending a position command, wait this long for the arm to
#   physically reach the target before sending the next command.
MOVE_SETTLE_TIME = 1.5    # seconds (adjust based on profile velocity)


# ── State machine ─────────────────────────────────────────────────────
class State(Enum):
    IDLE             = auto()
    MOVE_TO_SCAN_POSE = auto()
    SCANNING         = auto()
    APPROACHING      = auto()
    GRABBING         = auto()
    SORTING          = auto()
    DROPPING         = auto()
    DONE             = auto()


# ── Pretty logging ────────────────────────────────────────────────────
def log_state(state: State, msg: str = ""):
    icons = {
        State.IDLE:             "🏠",
        State.MOVE_TO_SCAN_POSE: "🔄",
        State.SCANNING:         "📷",
        State.APPROACHING:      "🎯",
        State.GRABBING:         "✊",
        State.SORTING:          "📦",
        State.DROPPING:         "📤",
        State.DONE:             "✅",
    }
    icon = icons.get(state, "❓")
    print(f"\n{'═' * 60}")
    print(f"  {icon}  STATE → {state.name}")
    if msg:
        print(f"     {msg}")
    print(f"{'═' * 60}")


def send_command(ser, arm: ArmIK, x: float, y: float, z: float, label: str = "",
                 viz=None):
    """Solve IK, send JSON over serial, wait for ACK, and update visualizer.

    Parameters
    ----------
    viz : ArmVisualizer or None
        If provided, update the 3-D plot with the new motor positions.
    """
    if label:
        print(f"\n  📍 {label}: target=({x:.1f}, {y:.1f}, {z:.1f}) cm")

    solution = arm.solve(x, y, z)
    cmd_json = json.dumps(solution)
    ser.write((cmd_json + "\n").encode())

    response = ser.readline().decode().strip()
    if response != "OK":
        print(f"  ⚠️  Unexpected response: {response}")

    # Update visualizer if available (needed for real serial mode)
    if viz is not None:
        viz.update_plot(solution)

    # Wait for the physical arm to reach the target position
    if USE_REAL_SERIAL:
        time.sleep(MOVE_SETTLE_TIME)

    return response


# send_partial() removed — two-step approach replaced with single move (see ADR 003)


def send_claw(ser, position: int, label: str = ""):
    """Send a claw (motor 5) position command over serial.

    This reads the last known IK solution's m1-m4 values and replaces m5
    with the desired claw position.  For simplicity, we send only the
    m5 update as a full 5-motor JSON (keeping m1-m4 at their last values).

    Parameters
    ----------
    ser : serial port
        The serial connection.
    position : int
        Dynamixel step value for the claw motor (0-4095).
    label : str
        Optional log label.
    """
    if label:
        print(f"  🦀  [CLAW] {label} (m5 → {position})")

    # We read the current positions first, then set only m5
    cmd = json.dumps({"cmd": "read_pos"})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode().strip()

    try:
        current = json.loads(resp)
    except (json.JSONDecodeError, TypeError):
        # If we can't read, send a safe command with centre values for m1-m4
        current = {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 2048, "m5": 2048}

    # Override only the claw motor
    current["m5"] = position
    cmd_json = json.dumps(current)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode().strip()
    if resp != "OK":
        print(f"  ⚠️  Claw response: {resp}")


def send_scan_pose(ser, viz=None):
    """Drive all five motors directly to SCAN_POSE step values.

    Uses raw joint positions (NOT IK) because SCAN_POSE is defined in
    Dynamixel step space.  Waits at least 1 second after the move for
    motion to settle.

    Parameters
    ----------
    ser : serial port
        The (mock or real) serial connection.
    viz : ArmVisualizer or None
        If provided, update the 3-D plot with the new motor positions.
    """
    print("  🔄  Moving arm to SCAN_POSE for vision scan...")
    cmd_json = json.dumps(SCAN_POSE)
    ser.write((cmd_json + "\n").encode())

    response = ser.readline().decode().strip()
    if response != "OK":
        print(f"  ⚠️  Unexpected SCAN_POSE response: {response}")

    if viz is not None:
        viz.update_plot(SCAN_POSE)

    # Allow motion to settle before capturing images
    time.sleep(max(1.0, MOVE_SETTLE_TIME))


def smooth_startup(ser, viz=None):
    """Move arm to SCAN_POSE on startup using a smooth Dynamixel velocity profile.

    Steps:
      1. Re-enable motor torque (safe after a 12V power cycle with USB still on).
      2. Set a moderate trapezoidal velocity profile — the firmware's built-in
         profile handles smooth acceleration and deceleration automatically,
         so no manual interpolation is needed.
      3. Send SCAN_POSE (joint-space step values) in one command.
      4. Wait for the arm to physically settle at SCAN_POSE.

    Parameters
    ----------
    ser : serial port
        The (mock or real) serial connection.
    viz : ArmVisualizer or None
        If provided, update the 3-D plot with the new motor positions.
    """
    # 0. Re-enable torque in case 12V power was cycled but USB stayed on
    print("  🔌  Re-enabling motor torque...")
    cmd = json.dumps({"cmd": "enable_torque"})
    ser.write((cmd + "\n").encode())
    ser.readline()  # consume ACK (may be empty on first boot)

    # 1. Set smooth motion profile (trapezoidal: ramp-up → cruise → ramp-down)
    print(f"  🎯  Setting motion profile  (vel={STARTUP_PROFILE_VEL}, acc={STARTUP_PROFILE_ACC})...")
    cmd = json.dumps({"cmd": "set_profile", "vel": STARTUP_PROFILE_VEL, "acc": STARTUP_PROFILE_ACC})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode().strip()
    print(f"       Profile response: {resp}")

    # 2. Send SCAN_POSE directly — the profile ensures smooth, controlled motion
    print("  🏠  Moving to SCAN_POSE (home position)...")
    cmd_json = json.dumps(SCAN_POSE)
    ser.write((cmd_json + "\n").encode())
    response = ser.readline().decode().strip()
    if response != "OK":
        print(f"  ⚠️  Unexpected SCAN_POSE response: {response}")

    if viz is not None:
        viz.update_plot(SCAN_POSE)

    # 3. Wait for the arm to physically reach SCAN_POSE before the loop starts
    time.sleep(max(2.0, MOVE_SETTLE_TIME))
    print("  ✅  Arm at SCAN_POSE — ready")


# ══════════════════════════════════════════════════════════════════════
#  SINGLE PICK-AND-PLACE CYCLE
# ══════════════════════════════════════════════════════════════════════
def run_sorting_cycle(ser, arm: ArmIK, detection: dict, vision: VisionBridge,
                      viz=None):
    """Execute one full pick-and-place cycle for a detected object.

    Parameters
    ----------
    ser : MockSerial or serial.Serial
        The (mock or real) serial connection to the OpenRB-150.
    arm : ArmIK
        The IK solver instance.
    detection : dict
        Camera output, e.g.::

            {"colour": "red", "x": 20.0, "y": 5.0, "z": 0.0}
    vision : VisionBridge
        The vision bridge instance (retained for future use).
    viz : ArmVisualizer or None
        Live 3-D visualizer (passed to movement commands).
    """
    colour = detection["colour"]
    obj_x  = detection["x"]
    obj_y  = detection["y"]
    obj_z  = detection["z"]

    # ── 1. IDLE ───────────────────────────────────────────────────────
    log_state(State.IDLE, f"Detection: {colour.upper()} ball at ({obj_x}, {obj_y}, {obj_z})")
    time.sleep(0.3)

    # ── 2. APPROACHING (single full move) ────────────────────────────
    # Two-step approach removed — wrist-mounted camera occludes ball during approach (see ADR 003)
    log_state(State.APPROACHING, "Moving directly to grab position")

    # Ensure claw is open before approaching
    send_claw(ser, CLAW_OPEN_POS, label="Ensuring claw is OPEN")

    # Single full move — descend to grab height in one motion
    send_command(
        ser, arm, obj_x, obj_y, GRAB_HEIGHT,
        label="Full approach to grab position",
        viz=viz,
    )

    # ── 3. GRABBING ──────────────────────────────────────────────────
    log_state(State.GRABBING, f"Closing claw on {colour.upper()} ball")
    
    # Optional pause for measurement (as requested by user 2026-04-25)
    print(f"\n  🎯  TARGET REACH: {obj_x:.1f} cm, TARGET Z: {GRAB_HEIGHT:.1f} cm")
    input("  📏  [MEASURE] Arm is at grab position. Measure height now, then press ENTER to close claw... ")

    print(f"  ✊  [CLAW] Closing... (dwell {GRAB_DWELL}s)")
    send_claw(ser, CLAW_CLOSED_POS, label="CLOSE grip")
    time.sleep(GRAB_DWELL)
    print("  ✊  [CLAW] Object secured")

    # Lift to clearance height before traversing
    send_command(
        ser, arm, obj_x, obj_y, CLEARANCE_HEIGHT,
        label="Lifting to clearance height",
        viz=viz,
    )

    # ── 4. RETURN HOME ───────────────────────────────────────────────
    log_state(State.SORTING, "Returning to HOME position before dropping")
    send_command(ser, arm, *HOME_POSITION, label="Return HOME (with ball)", viz=viz)

    # Small delay to let the arm stabilise at home before releasing
    time.sleep(0.5)

    # ── 5. DROPPING ──────────────────────────────────────────────────
    log_state(State.DROPPING, f"Releasing {colour.upper()} ball at HOME")
    print(f"  📤  [CLAW] Opening... (dwell {RELEASE_DWELL}s)")
    send_claw(ser, CLAW_OPEN_POS, label="OPEN grip (release)")
    time.sleep(RELEASE_DWELL)
    print("  📤  [CLAW] Object released at HOME")

    log_state(State.DONE, "Cycle complete ✅")


# ══════════════════════════════════════════════════════════════════════
#  M A I N   L O O P  (queue-based scan → process → rescan)
# ══════════════════════════════════════════════════════════════════════
def main():
    # ── Parse CLI arguments ───────────────────────────────────────────
    global USE_REAL_SERIAL, USE_REAL_CAMERA
    parser = argparse.ArgumentParser(
        description="4-DOF robotic arm pick-and-place controller"
    )
    parser.add_argument(
        "--real-serial",
        action="store_true",
        default=False,
        help="Connect to physical Dynamixel servos over serial (default: simulation mode)",
    )
    parser.add_argument(
        "--real-camera",
        action="store_true",
        default=False,
        help="Use OAK-D camera for vision (default: use simulated/recorded frames)",
    )
    args = parser.parse_args()
    USE_REAL_SERIAL = args.real_serial
    USE_REAL_CAMERA = args.real_camera

    print("╔══════════════════════════════════════════════════════════════╗")
    print("║        AUTONOMIA – Sorting Arm Master Controller           ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print(f"║  Serial : {'REAL → ' + SERIAL_PORT if USE_REAL_SERIAL else 'MOCK (no hardware)':<40s}   ║")
    print(f"║  Camera : {'REAL → OAK-D' if USE_REAL_CAMERA else 'FAKE (simulation)':<40s}   ║")
    print("╚══════════════════════════════════════════════════════════════╝\n")

    # ── Initialise IK solver ──────────────────────────────────────────
    arm = ArmIK()
    print(f"[INIT] IK solver ready  (L1={arm.L1}, L2={arm.L2}, L3={arm.L3} cm)")

    # ── Initialise 3-D visualiser ─────────────────────────────────────
    viz = ArmVisualizer()
    print("[INIT] 3-D visualiser ready")

    # ── Open serial connection ────────────────────────────────────────
    if USE_REAL_SERIAL:
        import serial
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=2)
        time.sleep(3)  # wait for OpenRB-150 to boot (increased from 2s)
        # Drain any boot messages from the firmware
        boot_msg = ""
        while ser.in_waiting:
            boot_msg += ser.readline().decode(errors="replace").strip() + " "
        if not boot_msg.strip():
            boot_msg = ser.readline().decode(errors="replace").strip()
        print(f"[INIT] OpenRB says: {boot_msg.strip()}")
        if "READY" not in boot_msg.upper():
            print("[INIT] ⚠️  Did not receive 'OK:READY' from OpenRB-150!")
            print("       Check: (1) firmware uploaded? (2) correct port? (3) baud rate?")
    else:
        ser = MockSerial(move_delay=1.0, visualizer=viz, anim_frames=30)

    # ── Initialise vision bridge ──────────────────────────────────────
    vision = VisionBridge(use_camera=USE_REAL_CAMERA)
    if not vision.open():
        print("[INIT] ❌ Vision bridge failed to open real camera.")
        print("       Check: (1) USB connection? (2) Re-plug the camera? (3) Power?")
        return  # Stop here instead of falling back to fake data

    # ── Clear any latched hardware errors before moving ──────────────
    if USE_REAL_SERIAL:
        print("\n[INIT] Clearing any latched hardware errors...")
        cmd = json.dumps({"cmd": "clear_errors"})
        ser.write((cmd + "\n").encode())
        resp = ser.readline().decode(errors="replace").strip()
        print(f"[INIT] clear_errors response: {resp}")

    # ── Go to SCAN_POSE on startup (smooth profile move) ─────────────
    print("\n[INIT] Moving to SCAN_POSE (home position) on startup...")
    smooth_startup(ser, viz=viz)

    # ── Continuous scan → sort → rescan loop ──────────────────────────
    IDLE_RESCAN_DELAY = 3       # seconds to wait between idle rescans
    scan_round = 0

    try:
        while True:
            scan_round += 1

            # ── MOVE_TO_SCAN_POSE ─────────────────────────────────────
            log_state(State.MOVE_TO_SCAN_POSE, f"Preparing for scan round {scan_round}")
            send_scan_pose(ser, viz=viz)

            # ── SCANNING ──────────────────────────────────────────────
            log_state(State.SCANNING, f"Scan round {scan_round}")
            time.sleep(SCAN_INTERVAL)   # wait SCAN_INTERVAL seconds before capturing frame
            detections = vision.scan_for_balls()

            if not detections:
                # No balls found — wait before rescanning.
                # Reset round counter so it never "runs out".
                scan_round = 0
                print("\n  📷  No objects found — workspace is clear")
                print(f"  ⏳ Waiting for balls... (rescanning in {IDLE_RESCAN_DELAY}s)")

                # Idle loop: keep camera feed + visualiser responsive
                wait_end = time.time() + IDLE_RESCAN_DELAY
                quit_requested = False
                while time.time() < wait_end:
                    # Update the OpenCV window so the user sees a live feed
                    key = cv2.waitKey(100) & 0xFF   # ~10 fps refresh, also pumps GUI events
                    if key == ord('q'):
                        quit_requested = True
                        break
                if quit_requested:
                    print("\n  ⛔  'q' pressed — shutting down...")
                    break
                continue

            # Reset round counter on a new batch of detections
            scan_round = 0

            # Pick only the first ball for this cycle to ensure fresh positions on the next scan
            detection = detections[0]
            print(f"\n[SCAN] Found {len(detections)} object(s). Processing the first: "
                  f"{detection['colour'].upper()} at ({detection['x']:.1f}, {detection['y']:.1f})")

            run_sorting_cycle(ser, arm, detection, vision, viz=viz)
            print("\n[DONE] Object processed — returning to SCAN_POSE for next cycle...")

            # In simulation mode, one pass is enough (fake data won't change)
            if not USE_REAL_CAMERA:
                print("[QUEUE] Simulation mode — exiting after one pass")
                break

    except KeyboardInterrupt:
        print(f"\n\n{'━' * 60}")
        print("  ⛔  KeyboardInterrupt received — shutting down gracefully...")
        print(f"{'━' * 60}")

    # Return arm to HOME before powering off
    try:
        log_state(State.IDLE, "Returning to HOME before shutdown")
        send_command(ser, arm, *HOME_POSITION, label="Shutdown HOME", viz=viz)
    except Exception as e:
        print(f"  ⚠️  Could not return to HOME: {e}")

    # ── Shutdown ──────────────────────────────────────────────────────
    print(f"\n{'━' * 60}")
    print("  🏁  Arm is at HOME.  Shutting down.")
    print(f"{'━' * 60}\n")

    vision.close()                # releases camera + destroys OpenCV windows
    cv2.destroyAllWindows()       # safety fallback
    ser.close()
    viz.close()                   # blocks until user closes the plot window


if __name__ == "__main__":
    main()
