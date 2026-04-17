"""
main.py
=======
Master sorting loop for the autonomous robotic arm.

Implements a state-machine that drives the arm through:
    IDLE → SCANNING → APPROACHING (80%) → APPROACHING (100%) → GRABBING →
    SORTING (lift + move to bin) → DROPPING → IDLE → (next in queue or rescan)

Camera integration
------------------
When ``USE_REAL_CAMERA = True``, uses ``VisionBridge`` to capture real
detections from the OAK-D camera and convert pixel positions to arm-frame
centimetres via a calibrated homography.

When ``USE_REAL_CAMERA = False``, the bridge returns canned fake detections
so the state machine and 3-D visualiser can be tested without hardware.

Uses 5 daisy-chained Dynamixel motors via ``mock_serial.MockSerial``
while hardware is unavailable.
Swap to ``serial.Serial`` once the OpenRB-150 is connected.
"""

import json
import time
from collections import deque
from enum import Enum, auto

import cv2

from pi_kinematics import ArmIK
from config import (
    BINS,
    HOME_POSITION,
    GRAB_HEIGHT,
    CLEARANCE_HEIGHT,
    GRAB_DWELL,
    RELEASE_DWELL,
    get_bin_coords,
)
from Simu.mock_serial import MockSerial
from Simu.visualizer import ArmVisualizer
from vision_bridge import VisionBridge

# ─── Toggles — flip these when real hardware arrives ──────────────────
USE_REAL_SERIAL = False
SERIAL_PORT     = "/dev/ttyACM0"
SERIAL_BAUD     = 115200

USE_REAL_CAMERA = True          # True → OAK-D camera, False → fake data


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


def send_command(ser, arm: ArmIK, x: float, y: float, z: float, label: str = ""):
    """Solve IK, send JSON over serial, and wait for ACK."""
    if label:
        print(f"\n  📍 {label}: target=({x:.1f}, {y:.1f}, {z:.1f}) cm")

    cmd_json = arm.solve_to_json(x, y, z)
    ser.write((cmd_json + "\n").encode())

    response = ser.readline().decode().strip()
    if response != "OK":
        print(f"  ⚠️  Unexpected response: {response}")
    return response


def send_partial(ser, arm: ArmIK, tx, ty, tz, pct, ox=0.0, oy=0.0, oz=0.0, label=""):
    """Solve partial IK, send JSON over serial, and wait for ACK."""
    if label:
        print(f"\n  📍 {label}: {int(pct*100)}% toward ({tx:.1f}, {ty:.1f}, {tz:.1f})")

    cmd_json = arm.partial_move_to_json(tx, ty, tz, pct, ox, oy, oz)
    ser.write((cmd_json + "\n").encode())

    response = ser.readline().decode().strip()
    if response != "OK":
        print(f"  ⚠️  Unexpected response: {response}")
    return response


# ══════════════════════════════════════════════════════════════════════
#  SINGLE PICK-AND-PLACE CYCLE
# ══════════════════════════════════════════════════════════════════════
def run_sorting_cycle(ser, arm: ArmIK, detection: dict, vision: VisionBridge):
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
    """
    colour = detection["colour"]
    obj_x  = detection["x"]
    obj_y  = detection["y"]
    obj_z  = detection["z"]

    # Home position as the "origin" for partial moves
    hx, hy, hz = HOME_POSITION

    # ── 1. IDLE ───────────────────────────────────────────────────────
    log_state(State.IDLE, f"Detection: {colour.upper()} block at ({obj_x}, {obj_y}, {obj_z})")
    time.sleep(0.3)

    # ── 2. APPROACHING (two-step visual servoing) ────────────────────
    log_state(State.APPROACHING, "Phase 1 — moving 80% toward object")

    # 80% move from current (home) position
    send_partial(
        ser, arm, obj_x, obj_y, obj_z, 0.80,
        hx, hy, hz,
        label="Partial approach (80%)",
    )

    # Visual servoing: take a correction image
    print("\n  📸  Taking correction image...")
    correction = vision.refine_detection(colour)
    if correction is not None:
        obj_x = correction["x"]
        obj_y = correction["y"]
        print(f"  📸  Corrected target → ({obj_x}, {obj_y})")
    else:
        print(f"  📸  {colour.upper()} object lost! Cancelling pickup.")
        send_command(ser, arm, *HOME_POSITION, label="Return HOME (Aborted)")
        return

    log_state(State.APPROACHING, "Phase 2 — moving to final grab position")

    # 100% move — descend to grab height
    send_command(
        ser, arm, obj_x, obj_y, GRAB_HEIGHT,
        label="Final approach (100%)",
    )

    # ── 3. GRABBING ──────────────────────────────────────────────────
    log_state(State.GRABBING, f"Closing claw on {colour.upper()} block")
    print(f"  ✊  [CLAW] Closing... (dwell {GRAB_DWELL}s)")
    time.sleep(GRAB_DWELL)
    print("  ✊  [CLAW] Object secured")

    # Lift to clearance height before traversing
    send_command(
        ser, arm, obj_x, obj_y, CLEARANCE_HEIGHT,
        label="Lifting to clearance height",
    )

    # ── 4. SORTING ───────────────────────────────────────────────────
    bin_coords = get_bin_coords(colour)
    log_state(State.SORTING, f"Moving to {colour.upper()}_BIN at {bin_coords}")

    send_command(
        ser, arm, *bin_coords,
        label=f"Move to {colour.upper()}_BIN",
    )

    # ── 5. DROPPING ──────────────────────────────────────────────────
    log_state(State.DROPPING, f"Releasing object into {colour.upper()} bin")
    print(f"  📤  [CLAW] Opening... (dwell {RELEASE_DWELL}s)")
    time.sleep(RELEASE_DWELL)
    print("  📤  [CLAW] Object released")

    # ── 6. Return HOME ───────────────────────────────────────────────
    log_state(State.IDLE, "Returning to HOME position")
    send_command(ser, arm, *HOME_POSITION, label="Return HOME")

    log_state(State.DONE, "Cycle complete ✅")


# ══════════════════════════════════════════════════════════════════════
#  M A I N   L O O P  (queue-based scan → process → rescan)
# ══════════════════════════════════════════════════════════════════════
def main():
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
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        time.sleep(2)  # wait for OpenRB-150 to boot
        boot_msg = ser.readline().decode().strip()
        print(f"[INIT] OpenRB says: {boot_msg}")
    else:
        ser = MockSerial(move_delay=1.0, visualizer=viz, anim_frames=30)

    # ── Initialise vision bridge ──────────────────────────────────────
    vision = VisionBridge(use_camera=USE_REAL_CAMERA)
    if not vision.open():
        print("[INIT] ❌ Vision bridge failed — falling back to fake data")
        vision = VisionBridge(use_camera=False)
        vision.open()

    # ── Go to HOME on startup ────────────────────────────────────────
    print("\n[INIT] Moving to HOME position on startup...")
    send_command(ser, arm, *HOME_POSITION, label="Startup HOME")

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
                print(f"\n  📷  No objects found — workspace is clear")
                print(f"  ⏳ Waiting for balls... (rescanning in {IDLE_RESCAN_DELAY}s)")

                # Idle loop: keep camera feed + visualiser responsive
                wait_end = time.time() + IDLE_RESCAN_DELAY
                while time.time() < wait_end:
                    # Update the OpenCV window so the user sees a live feed
                    cv2.waitKey(100)   # ~10 fps refresh, also pumps GUI events
                continue

            # Reset round counter on a new batch of detections
            scan_round = 0

            # Build a processing queue
            queue = deque(detections)
            total = len(queue)

            print(f"\n[QUEUE] {total} object(s) queued for sorting")
            for i, det in enumerate(queue, 1):
                print(f"  {i}. {det['colour'].upper():5s}  "
                      f"at ({det['x']:6.1f}, {det['y']:6.1f}, {det['z']:4.1f}) cm")

            # ── Process the queue ─────────────────────────────────────
            cycle_num = 0
            while queue:
                cycle_num += 1
                detection = queue.popleft()

                print(f"\n{'▓' * 60}")
                print(f"  CYCLE {cycle_num}/{total}  "
                      f"({len(queue)} remaining)")
                print(f"{'▓' * 60}")

                run_sorting_cycle(ser, arm, detection, vision)

            print(f"\n[QUEUE] All objects processed — rescanning workspace...")

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
            send_command(ser, arm, *HOME_POSITION, label="Shutdown HOME")
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
