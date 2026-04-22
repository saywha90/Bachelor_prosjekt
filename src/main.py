"""
main.py
=======
Master sorting loop for the autonomous robotic arm.

Implements a state-machine that drives the arm through:
    IDLE → SCANNING → APPROACHING (80%) → APPROACHING (100%) → GRABBING →
    SORTING (lift + move to bin) → DROPPING → IDLE → (next in queue or rescan)

Camera integration
------------------
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
)
from simulation.mock_serial import MockSerial
from simulation.visualizer import ArmVisualizer
from ik.vision_bridge import VisionBridge

logger = logging.getLogger(__name__)

# ─── Serial / connection settings ─────────────────────────────────────
SERIAL_PORT     = "/dev/cu.usbmodem101"
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
    IDLE        = auto()
    SCANNING    = auto()
    APPROACHING = auto()
    GRABBING    = auto()
    SORTING     = auto()
    DROPPING    = auto()
    DONE        = auto()


# ── Pretty logging ────────────────────────────────────────────────────
def log_state(state: State, msg: str = ""):
    icons = {
        State.IDLE:        "🏠",
        State.SCANNING:    "📷",
        State.APPROACHING: "🎯",
        State.GRABBING:    "✊",
        State.SORTING:     "📦",
        State.DROPPING:    "📤",
        State.DONE:        "✅",
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


def send_partial(ser, arm: ArmIK, tx, ty, tz, pct, ox=0.0, oy=0.0, oz=0.0, label="",
                 viz=None):
    """Solve partial IK, send JSON over serial, wait for ACK, and update visualizer."""
    if label:
        print(f"\n  📍 {label}: {int(pct*100)}% toward ({tx:.1f}, {ty:.1f}, {tz:.1f})")

    solution = arm.calculate_partial_move(tx, ty, tz, pct, ox, oy, oz)
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


def smooth_startup(ser, arm: ArmIK, viz=None):
    """Gradually ramp the arm from its current position to HOME on startup.

    Steps:
      1. Set a slow motion profile on the firmware.
      2. Read current motor positions from the firmware.
      3. Interpolate from current positions to HOME in 10 steps.
      4. Restore normal motion profile.

    Falls back to a direct (but slow-profile) HOME command if reading
    current positions fails.
    """
    NUM_STEPS = 10
    STEP_DELAY = 0.15  # seconds between interpolation steps

    # 0. Send an explicit torque command in case 12V power was cycled but USB stayed on
    print("  🔌  Re-enabling motor torque...")
    cmd = json.dumps({"cmd": "enable_torque"})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode().strip()

    # 1. Set slow startup profile
    print("  🐢  Setting slow startup profile...")
    cmd = json.dumps({"cmd": "set_profile", "vel": 30, "acc": 10})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode().strip()
    print(f"       Profile response: {resp}")

    # 2. Read current motor positions
    print("  📖  Reading current motor positions...")
    cmd = json.dumps({"cmd": "read_pos"})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode().strip()

    # Compute HOME target positions
    home_positions = arm.solve(*HOME_POSITION)

    # Try to parse current positions
    current_positions = None
    try:
        current_positions = json.loads(resp)
        # Validate that all motor keys exist
        for key in ("m1", "m2", "m3", "m4", "m5"):
            if key not in current_positions:
                raise ValueError(f"Missing key {key} in response")
        print(f"       Current positions: {current_positions}")
    except (json.JSONDecodeError, ValueError, TypeError) as e:
        print(f"  ⚠️  Could not parse positions ({e}) — falling back to direct HOME")
        # Fall back: just send HOME with slow profile already set
        cmd_json = json.dumps(home_positions)
        ser.write((cmd_json + "\n").encode())
        resp = ser.readline().decode().strip()
        print(f"       HOME response: {resp}")
        # Restore normal profile
        cmd = json.dumps({"cmd": "set_profile", "vel": 80, "acc": 20})
        ser.write((cmd + "\n").encode())
        ser.readline()
        return

    # 3. Interpolate from current to HOME in NUM_STEPS steps
    print(f"  🔄  Interpolating to HOME in {NUM_STEPS} steps...")
    motor_keys = ["m1", "m2", "m3", "m4", "m5"]

    for step in range(1, NUM_STEPS + 1):
        t = step / NUM_STEPS  # 0.1, 0.2, ..., 1.0
        interp = {}
        for key in motor_keys:
            start_val = current_positions[key]
            end_val = home_positions[key]
            interp[key] = int(round(start_val + (end_val - start_val) * t))

        cmd_json = json.dumps(interp)
        ser.write((cmd_json + "\n").encode())
        resp = ser.readline().decode().strip()
        if resp != "OK":
            print(f"  ⚠️  Step {step}/{NUM_STEPS} response: {resp}")
        # Update visualizer during startup ramp
        if viz is not None:
            viz.update_plot(interp)
        time.sleep(STEP_DELAY)

    print("  ✅  Startup ramp complete — arm at HOME")

    # 4. Restore normal motion profile
    print("  🚀  Restoring normal motion profile...")
    cmd = json.dumps({"cmd": "set_profile", "vel": 80, "acc": 20})
    ser.write((cmd + "\n").encode())
    resp = ser.readline().decode().strip()
    print(f"       Profile response: {resp}")


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
        The vision bridge (used for visual-servoing correction image).
    viz : ArmVisualizer or None
        Live 3-D visualizer (passed to movement commands).
    """
    colour = detection["colour"]
    obj_x  = detection["x"]
    obj_y  = detection["y"]
    obj_z  = detection["z"]

    # Home position as the "origin" for partial moves
    hx, hy, hz = HOME_POSITION

    # ── 1. IDLE ───────────────────────────────────────────────────────
    log_state(State.IDLE, f"Detection: {colour.upper()} ball at ({obj_x}, {obj_y}, {obj_z})")
    time.sleep(0.3)

    # ── 2. APPROACHING (two-step visual servoing) ────────────────────
    log_state(State.APPROACHING, "Phase 1 — moving 80% toward object")

    # Ensure claw is open before approaching
    send_claw(ser, CLAW_OPEN_POS, label="Ensuring claw is OPEN")

    # 80% move from current (home) position — stay at safe hover height
    send_partial(
        ser, arm, obj_x, obj_y, APPROACH_HEIGHT, 0.80,
        hx, hy, hz,
        label="Partial approach (80%)",
        viz=viz,
    )

    # Visual servoing correction (DISABLED for stability)
    # correction = vision.refine_detection(colour) ...
    print("  📸  Skipping correction to maintain grab alignment stability.")

    log_state(State.APPROACHING, "Phase 2 — moving to final grab position")

    # 100% move — descend to grab height
    send_command(
        ser, arm, obj_x, obj_y, GRAB_HEIGHT,
        label="Final approach (100%)",
        viz=viz,
    )

    # ── MEASUREMENT PAUSE ─────────────────────────────────────────────
    import math as _math
    _reach = _math.sqrt(obj_x**2 + obj_y**2)
    print(f"\n{'='*58}")
    print("  ⏸  PAUSED AT GRAB POSITION")
    print(f"{'='*58}")
    print(f"  Vision target : x={obj_x:.1f}, y={obj_y:.1f}, z={GRAB_HEIGHT}")
    print(f"  Horiz. reach  : {_reach:.1f} cm from shoulder")
    print(f"  CAMERA_OFFSET_X = {CAMERA_OFFSET_X:.1f} cm  (from config/arm.py)")
    print(f"{'─'*58}")
    print("  📏 MEASURE NOW:")
    print("     1. Distance from SHOULDER JOINT to the BALL (cm)")
    print("     2. Distance from SHOULDER JOINT to the CLAW TIP (cm)")
    print("     3. Claw height above table (cm)")
    print(f"{'─'*58}")
    print("  If the claw is SHORT of the ball, INCREASE CAMERA_OFFSET_X")
    print("  If the claw OVERSHOOTS the ball, DECREASE CAMERA_OFFSET_X")
    print(f"{'='*58}")
    print("  📷  Showing live vision feed for verification...")
    print("  ⌨️   Press [ENTER] in this terminal to continue or [Q] in camera window to skip...")
    
    # Simple loop to keep camera feed alive during pause
    while True:
        vision.scan_for_balls() # This updates the CV2 window
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break
        # We use a non-blocking check for Enter key (via input with timeout is hard in standard python, 
        # so we'll just wait for the user to hit Enter in the terminal which will break the main flow)
        print("  Press ENTER to continue...", end="\r")
        # In this specific CLI script, we'll just use a normal input() but 
        # the user can see the window update in the background.
        break 

    input("  Press ENTER to proceed with GRAB...")
    print()

    # ── 3. GRABBING ──────────────────────────────────────────────────
    log_state(State.GRABBING, f"Closing claw on {colour.upper()} ball")
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

    # ── Go to HOME on startup (smooth ramp to avoid jerking) ─────────
    print("\n[INIT] Moving to HOME position on startup (smooth ramp)...")
    smooth_startup(ser, arm, viz=viz)

    # ── Continuous scan → sort → rescan loop ──────────────────────────
    IDLE_RESCAN_DELAY = 3       # seconds to wait between idle rescans
    scan_round = 0

    try:
        while True:
            scan_round += 1

            # ── SCANNING ──────────────────────────────────────────────
            log_state(State.SCANNING, f"Scan round {scan_round}")
            detections = vision.scan_for_balls()

            if not detections:
                # No balls found — stay at HOME and wait before rescanning.
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
            print("\n[DONE] Object processed — rescanning workspace for updated positions...")

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
