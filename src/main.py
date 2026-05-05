from __future__ import annotations

"""
main.py
=======
Master sorting loop for the autonomous robotic arm.

Implements a state-machine that drives the arm through:
    HOME → MOVE_TO_SCAN_POSE → SCANNING → APPROACHING → GRABBING →
    VERIFY_GRIP → SORTING (lift + move to bin) → DROPPING →
    MOVE_TO_SCAN_POSE → (rescan or next from queue)

Camera integration
-------------------
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
import serial

from ik.solver import ArmIK
from config.arm import (
    compute_grab_height,
    compute_wrist_correction,
    get_transport_route,
    RouteCalibrationError,
    load_transport_route_calibration,
    load_height_calibration,
    CLEARANCE_HEIGHT,
    GRAB_DWELL,
    RELEASE_DWELL,
    SCAN_POSE,
    STARTUP_PROFILE_VEL,
    STARTUP_PROFILE_ACC,
    SCAN_INTERVAL,
    GRIP_VERIFY_TOLERANCE,
    GRIP_LOAD_THRESHOLD,
    MAX_PICK_RETRIES,
    VERIFY_HEIGHT,
    GRIP_CURRENT_LIMIT,
    GRIP_PROFILE_VEL,
    GRIP_PROFILE_ACC,
    GRIP_POLL_INTERVAL,
    GRIP_TIMEOUT,
    GRIP_LOAD_DETECT,
    GRIP_POSITION_STALL,
    GRIP_EXTRA_CLOSE,
    DEFAULT_PROFILE_VEL,
    DEFAULT_PROFILE_ACC,
    M5_DEFAULT_CURRENT_LIMIT,
    CLAW_OPEN_POS,
    CLAW_CLOSED_POS,
)
from simulation.mock_serial import MockSerial
from ik.vision_bridge import VisionBridge

logger = logging.getLogger(__name__)

USE_REAL_SERIAL = False
USE_REAL_CAMERA = False


# ══════════════════════════════════════════════════════════════════════
#  CYCLE TIMER — timing instrumentation for sorting cycles
# ══════════════════════════════════════════════════════════════════════
class CycleTimer:
    """Tracks timing for each phase of a sorting cycle."""

    def __init__(self):
        self._timestamps: dict[str, float] = {}
        self._phase_times: dict[str, float] = {}
        self._current_phase: str | None = None
        self._cycle_start: float = 0.0
        self._history: list[dict[str, float]] = []

    def start_cycle(self):
        """Mark the beginning of a new sorting cycle."""
        self._timestamps.clear()
        self._phase_times.clear()
        self._current_phase = None
        self._cycle_start = time.perf_counter()

    def start_phase(self, name: str):
        """Begin timing a named phase."""
        now = time.perf_counter()
        if self._current_phase:
            self._phase_times[self._current_phase] = now - self._timestamps[self._current_phase]
        self._timestamps[name] = now
        self._current_phase = name

    def end_cycle(self) -> dict[str, float]:
        """End the cycle, record final phase, return all phase timings."""
        now = time.perf_counter()
        if self._current_phase:
            self._phase_times[self._current_phase] = now - self._timestamps[self._current_phase]
        self._phase_times["total"] = now - self._cycle_start
        self._history.append(self._phase_times.copy())
        return self._phase_times

    def print_cycle_summary(self, timings: dict[str, float]):
        """Print a formatted summary of the cycle timings."""
        parts = []
        for phase, duration in timings.items():
            if phase != "total":
                parts.append(f"{phase}: {duration:.2f}s")
        total = timings.get("total", 0)
        summary = ", ".join(parts)
        print(f"\n  ⏱️  Cycle time: {total:.2f}s ({summary})")

    def print_session_summary(self):
        """Print average timings across all completed cycles."""
        if not self._history:
            print("\n  ⏱️  No completed cycles to summarize.")
            return

        # Collect all phase names
        all_phases = set()
        for h in self._history:
            all_phases.update(h.keys())

        print(f"\n{'='*60}")
        print(f"  ⏱️  SESSION TIMING SUMMARY ({len(self._history)} cycles)")
        print(f"{'='*60}")

        for phase in sorted(all_phases):
            values = [h[phase] for h in self._history if phase in h]
            if values:
                avg = sum(values) / len(values)
                min_v = min(values)
                max_v = max(values)
                print(f"  {phase:>12}: avg {avg:.2f}s  "
                      f"(min {min_v:.2f}s, max {max_v:.2f}s, n={len(values)})")
        print(f"{'='*60}")


# ─── Serial / connection settings ─────────────────────────────────────
SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD     = 115200

# ─── Claw motor positions (Dynamixel steps) ───────────────────────────
# Imported from config.arm (single source of truth).

# ─── Movement settling time (seconds) ────────────────────────────────
#   After sending a position command, wait this long for the arm to
#   physically reach the target before sending the next command.
MOVE_SETTLE_TIME = 1.5    # seconds (MUST be long enough for the arm to physically reach the target)


# ── State machine ─────────────────────────────────────────────────────
class State(Enum):
    IDLE             = auto()
    MOVE_TO_SCAN_POSE = auto()
    SCANNING         = auto()
    APPROACHING      = auto()
    GRABBING         = auto()
    VERIFY_GRIP      = auto()
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
        State.VERIFY_GRIP:      "🔍",
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
                 viz=None, claw_override=None, m4_offset: int = 0,
                 skip_sag: bool = False, settle_time: float | None = None):
    """Solve IK, send JSON over serial, wait for ACK, and update visualizer.

    Parameters
    ----------
    viz : ArmVisualizer or None
        If provided, update the 3-D plot with the new motor positions.
    m4_offset : int
        Dynamixel step offset to add to the IK-computed m4 (wrist tilt).
        Used for runtime wrist correction learned during touch calibration.
        Clamped to [500, 3500] after application.
    skip_sag : bool
        If True, bypass the IK solver's internal sag compensation.
        Use when the target Z already accounts for real-world sag
        (e.g. touch-calibrated grab heights).
    """
    if label:
        print(f"\n  📍 {label}: target=({x:.1f}, {y:.1f}, {z:.1f}) cm")

    solution = arm.solve(x, y, z, skip_sag=skip_sag)
    if m4_offset:
        solution["m4"] = max(500, min(3500, solution["m4"] + m4_offset))
    if claw_override is not None:
        solution["m5"] = claw_override
    
    cmd_json = json.dumps(solution)
    ser.write((cmd_json + "\n").encode())

    response = ser.readline().decode().strip()
    if response != "OK":
        logger.warning("Unexpected response in send_command: %s", response)

    # Update visualizer if available (needed for real serial mode)
    if viz is not None:
        viz.update_plot(solution)

    # Wait for the physical arm to reach the target position
    if USE_REAL_SERIAL:
        if settle_time is not None:
            time.sleep(settle_time)
        else:
            time.sleep(MOVE_SETTLE_TIME)

    return solution


def send_strict_solution(ser, solution: dict, label: str = "", viz=None,
                         settle_time: float | None = None) -> dict:
    """Send prevalidated strict IK motor commands over serial."""
    commands = solution["commands"].copy()
    validation = solution.get("validation", {})
    pose = validation.get("pose", {})
    intent = validation.get("intent", "strict")

    if label:
        if all(key in pose for key in ("x", "y", "z")):
            print(
                f"\n  📍 {label}: intent={intent}, "
                f"target=({pose['x']:.1f}, {pose['y']:.1f}, {pose['z']:.1f}) cm"
            )
        else:
            print(f"\n  📍 {label}: intent={intent}")

    ser.write((json.dumps(commands) + "\n").encode())

    response = ser.readline().decode().strip()
    if response != "OK":
        logger.warning("Unexpected response in send_strict_solution: %s", response)

    if viz is not None:
        viz.update_plot(commands)

    if USE_REAL_SERIAL:
        if settle_time is not None:
            time.sleep(settle_time)
        else:
            time.sleep(MOVE_SETTLE_TIME)

    return commands


def prevalidate_transport_plan(arm: ArmIK, colour: str, pickup_pose: dict) -> list[tuple[str, dict]]:
    """Strictly solve all pickup recovery and destination route waypoints upfront."""
    route_cal = load_transport_route_calibration(require_route_schema=True)
    route = get_transport_route(colour)
    plan: list[tuple[str, dict]] = []

    pickup_solution = arm.solve_strict(pickup_pose, intent="pickup")
    plan.append(("pickup", pickup_solution))

    pickup_recovery_pose = pickup_pose.copy()
    pickup_recovery_pose["z"] = max(CLEARANCE_HEIGHT, pickup_pose["z"])
    pickup_recovery_pose["m5"] = CLAW_OPEN_POS
    plan.append(("pickup_recovery_clearance", arm.solve_strict(pickup_recovery_pose, intent="carry")))

    for name, route_pose in route:
        intent = "carry" if name == "front_neutral" else "rear_place"
        strict_pose = route_pose.as_strict_pose(m5=CLAW_CLOSED_POS)
        if name == "rear_transfer" or "." in name:
            strict_pose["rear_base_yaw_limit_deg"] = route_cal.rear_base_yaw_limit_deg
        plan.append((name, arm.solve_strict(strict_pose, intent=intent)))

    return plan


# send_partial() removed — two-step approach replaced with single move (see ADR 003)


def send_claw(ser, last_solution: dict, position: int, label: str = ""):
    """Send a claw (motor 5) position command over serial.

    This uses the last known IK solution's m1-m4 values and replaces m5
    with the desired claw position.

    Parameters
    ----------
    ser : serial port
        The serial connection.
    last_solution : dict
        The last target positions sent to the arm (m1-m4).
    position : int
        Dynamixel step value for the claw motor (0-4095).
    label : str
        Optional log label.
    """
    if label:
        print(f"  🦀  [CLAW] {label} (m5 → {position})")

    # Use the last known positions to prevent sudden jerks
    current = last_solution.copy() if last_solution else {"m1": 2048, "m2": 2048, "m3": 2048, "m4": 1911}

    # Override only the claw motor
    current["m5"] = position
    cmd_json = json.dumps(current)
    ser.write((cmd_json + "\n").encode())
    resp = ser.readline().decode().strip()
    if resp != "OK":
        logger.warning("Unexpected claw response: %s", resp)


def _read_motor_data(ser, cmd: str) -> dict | None:
    """Send a read command and parse the JSON motor-data response."""
    if ser is None:
        return None
    payload = json.dumps({"cmd": cmd}) + "\n"
    ser.write(payload.encode())
    try:
        raw = ser.readline().decode().strip()
        return json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        logger.warning("Failed to read motor data (cmd=%s): %s", cmd, e)
        return None


def read_positions(ser) -> dict | None:
    """Read current positions from all motors via firmware."""
    return _read_motor_data(ser, "read_pos")


def read_load(ser) -> dict | None:
    """Read current load from all motors via firmware."""
    return _read_motor_data(ser, "read_load")


def read_current(ser) -> dict | None:
    """Read present current from all motors via firmware."""
    return _read_motor_data(ser, "read_current")


def send_raw_command(ser, cmd_dict: dict) -> str:

    """Send a raw JSON command dict over serial and return the response.

    Unlike ``send_command()`` (which is IK-aware), this sends an arbitrary
    JSON object — used for firmware meta-commands like ``set_current_limit``,
    ``set_profile``, ``read_load``, etc.
    """
    payload = json.dumps(cmd_dict) + "\n"
    ser.write(payload.encode())
    try:
        resp = ser.readline().decode(errors="replace").strip()
    except (serial.SerialException, serial.SerialTimeoutException, OSError) as e:
        logger.warning("Serial read failed in send_raw_command: %s", e)
        resp = ""
    return resp


def adaptive_grip(ser, last_solution: dict) -> bool:
    """Gradually close the claw until resistance is detected.

    Returns True if an object was gripped, False if closed on air.

    Strategy:
      1. Set a low current limit on M5 to protect the 3D-printed claw
      2. Set slow profile velocity for gentle closing
      3. Command CLAW_CLOSED_POS as goal position
      4. Poll read_load every GRIP_POLL_INTERVAL
      5. When load exceeds GRIP_LOAD_DETECT, object is contacted
      6. Close GRIP_EXTRA_CLOSE steps past contact for secure hold
      7. Restore normal current limit and profile velocity
    """
    print("  🤏  [ADAPTIVE GRIP] Starting adaptive grip sequence")
    gripped = False
    try:
        # 1. Cap M5 current to protect the claw
        print(f"  🤏  [ADAPTIVE GRIP] Setting M5 current limit to {GRIP_CURRENT_LIMIT} mA")
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 5, "value": GRIP_CURRENT_LIMIT})

        # 2. Slow down the closing motion
        print(f"  🤏  [ADAPTIVE GRIP] Setting slow profile (vel={GRIP_PROFILE_VEL}, acc={GRIP_PROFILE_ACC})")
        send_raw_command(ser, {"cmd": "set_profile", "vel": GRIP_PROFILE_VEL, "acc": GRIP_PROFILE_ACC})

        # 3. Command m5 to CLAW_CLOSED_POS while maintaining previous goal positions for m1-m4
        # to prevent the arm from springing up if it was pushing against the desk
        if not last_solution:
            logger.warning("[ADAPTIVE GRIP] No last_solution provided — falling back to verify_grip()")
            return verify_grip(ser, CLAW_CLOSED_POS)

        goal = last_solution.copy()
        goal["m5"] = CLAW_CLOSED_POS
        ser.write((json.dumps(goal) + "\n").encode())
        ser.readline()  # consume ACK

        # 4. Polling loop — watch load + position for contact detection
        start_time = time.time()
        
        # Read initial position for stall detection baseline
        positions = read_positions(ser)
        prev_pos = int(positions.get("m5", last_solution.get("m5", 0))) if positions else int(last_solution.get("m5", 0))
        contact_detected = False
        contact_position = None

        while True:
            time.sleep(GRIP_POLL_INTERVAL)
            elapsed = time.time() - start_time

            # Timeout guard
            if elapsed > GRIP_TIMEOUT:
                logger.warning("[ADAPTIVE GRIP] Timeout after %.1fs", GRIP_TIMEOUT)
                break

            # Read load
            loads = read_load(ser)
            cur_positions = read_positions(ser)

            if loads is None or cur_positions is None:
                continue  # skip this poll cycle on read failure

            m5_load = abs(int(loads.get("m5", 0)))
            m5_pos = int(cur_positions.get("m5", 0))

            # Check for object contact via load
            if m5_load >= GRIP_LOAD_DETECT:
                print(f"  🤏  [ADAPTIVE GRIP] Contact detected! load={m5_load}, pos={m5_pos}")
                contact_detected = True
                contact_position = m5_pos
                break

            # Check for stall (position not changing)
            # Only check after 0.4s to allow the motor to accelerate from a dead stop
            if elapsed > 0.4 and abs(m5_pos - prev_pos) <= GRIP_POSITION_STALL:
                # Could be stalled against object or at end of travel
                print(f"  🤏  [ADAPTIVE GRIP] Stall detected at pos={m5_pos} (prev={prev_pos})")
                contact_detected = True
                contact_position = m5_pos
                break

            prev_pos = m5_pos

        # 5. If contact was detected, close a bit more for a secure hold
        if contact_detected and contact_position is not None:
            secure_pos = min(contact_position + GRIP_EXTRA_CLOSE, CLAW_CLOSED_POS)
            print(f"  🤏  [ADAPTIVE GRIP] Securing grip: closing to {secure_pos} "
                  f"(contact at {contact_position} + {GRIP_EXTRA_CLOSE} extra)")
            goal["m5"] = secure_pos
            ser.write((json.dumps(goal) + "\n").encode())
            ser.readline()  # consume ACK
            time.sleep(0.3)  # brief dwell for the extra close

        # 6. Check final position to determine if we actually gripped something
        final_positions = read_positions(ser)
        if final_positions and "m5" in final_positions:
            final_m5 = int(final_positions["m5"])
            if abs(final_m5 - CLAW_CLOSED_POS) <= GRIP_VERIFY_TOLERANCE:
                logger.warning("[ADAPTIVE GRIP] Claw closed on air (pos=%d, "
                      "target=%d, tol=%d)", final_m5, CLAW_CLOSED_POS, GRIP_VERIFY_TOLERANCE)
                gripped = False
            else:
                print(f"  ✅  [ADAPTIVE GRIP] Object gripped! (pos={final_m5}, "
                      f"blocked {abs(CLAW_CLOSED_POS - final_m5)} steps from closed)")
                gripped = True
        else:
            # Can't read position — fall back to verify_grip
            logger.warning("[ADAPTIVE GRIP] Cannot read final position — falling back to verify_grip()")
            gripped = verify_grip(ser, CLAW_CLOSED_POS)

    except Exception as e:
        logger.error("[ADAPTIVE GRIP] Error during adaptive grip: %s", e)
        logger.warning("[ADAPTIVE GRIP] Falling back to verify_grip()")
        try:
            gripped = verify_grip(ser, CLAW_CLOSED_POS)
        except Exception:
            gripped = False

    finally:
        # Restoring profile velocity/acceleration, but keeping M5 current limit low to maintain gentle grip
        print(f"  🤏  [ADAPTIVE GRIP] Restoring profile (vel={DEFAULT_PROFILE_VEL}, acc={DEFAULT_PROFILE_ACC})")
        try:
            send_raw_command(ser, {"cmd": "set_profile", "vel": DEFAULT_PROFILE_VEL, "acc": DEFAULT_PROFILE_ACC})
        except Exception as e:
            logger.warning("[ADAPTIVE GRIP] Failed to restore profile: %s", e)

    return gripped


def verify_grip(ser, claw_closed_pos: int) -> bool:
    """
    Check if the claw actually gripped something.

    Two checks:
    1. Load check: if claw motor shows significant load, something is resisting → grip confirmed
    2. Position check: if claw reached CLAW_CLOSED_POS (within tolerance),
       it closed on empty air → no grip

    Returns True if grip is confirmed, False if grip failed.
    """
    # Check 1: Load-based verification (Primary)
    loads = read_load(ser)
    if loads and "m5" in loads:
        claw_load = abs(loads["m5"])
        if claw_load >= GRIP_LOAD_THRESHOLD:
            print(f"  ✅ Grip check passed (load): claw load = {claw_load}")
            return True
        else:
            logger.warning("Grip check warning (load): claw load = %s "
                  "(below threshold %s)", claw_load, GRIP_LOAD_THRESHOLD)

    # Check 2: Position-based verification (Fallback)
    positions = read_positions(ser)
    if positions and "m5" in positions:
        claw_pos = positions["m5"]
        if abs(claw_pos - claw_closed_pos) <= GRIP_VERIFY_TOLERANCE:
            logger.warning("Grip check FAILED (position): claw at %s, "
                  "target was %s (within %s tolerance)", claw_pos, claw_closed_pos, GRIP_VERIFY_TOLERANCE)
            return False
        else:
            print(f"  ✅ Grip check passed (position): claw at {claw_pos}, "
                  f"blocked {abs(claw_closed_pos - claw_pos)} steps from closed")
            return True

    # Fallback: assume grip succeeded if we can't read anything
    logger.warning("Grip verification unavailable (no sensor data), assuming success")
    return True


def send_scan_pose(ser, viz=None, claw_override=None, settle_time: float | None = None):
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
    claw_override : int or None
        If provided, overrides the m5 (claw) value in SCAN_POSE.
    """
    if claw_override is None:
        print("  🔄  Moving arm to SCAN_POSE for vision scan...")
    else:
        print("  📦  Moving arm to SCAN_POSE with payload...")

    pose = SCAN_POSE.copy()
    if claw_override is not None:
        pose["m5"] = claw_override

    cmd_json = json.dumps(pose)
    ser.write((cmd_json + "\n").encode())

    response = ser.readline().decode().strip()
    if response != "OK":
        logger.warning("Unexpected SCAN_POSE response: %s", response)

    if viz is not None:
        viz.update_plot(pose)

    # Allow motion to settle before capturing images
    if settle_time is not None:
        if settle_time > 0:
            time.sleep(settle_time)
    else:
        time.sleep(max(1.0, MOVE_SETTLE_TIME))

    return pose


def verify_scan_pose_before_scan(ser, vision: VisionBridge) -> bool:
    """Verify the arm is at the calibrated SCAN_POSE before vision capture.

    The wrist-mounted camera homography is only valid from the scan pose used
    during calibration.  This helper reads the current motor positions and
    delegates the tolerance check to ``VisionBridge.verify_pose()``.  If motor
    positions cannot be read, scanning continues with a warning rather than
    crashing the main loop.
    """
    positions = read_positions(ser)
    if positions is None:
        logger.warning(
            "[SCAN] Could not read motor positions before scan; continuing without pose verification"
        )
        return False

    if vision.verify_pose(positions):
        logger.info("[SCAN] SCAN_POSE verified before camera capture")
        return True

    logger.warning(
        "[SCAN] Arm is not within calibrated SCAN_POSE tolerance before camera capture; continuing scan"
    )
    return False


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
        logger.warning("Unexpected SCAN_POSE response during startup: %s", response)

    if viz is not None:
        viz.update_plot(SCAN_POSE)

    # 3. Wait for the arm to physically reach SCAN_POSE before the loop starts
    time.sleep(max(2.0, MOVE_SETTLE_TIME))
    print("  ✅  Arm at SCAN_POSE — ready")


# ══════════════════════════════════════════════════════════════════════
#  SINGLE PICK-AND-PLACE CYCLE
# ══════════════════════════════════════════════════════════════════════
def run_sorting_cycle(ser, arm: ArmIK, detection: dict, vision: VisionBridge,
                      viz=None, timer: CycleTimer | None = None) -> bool:
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
    timer : CycleTimer or None
        If provided, phases will be timed and recorded.

    Returns
    -------
    bool
        True if the cycle completed successfully, False if grip verification failed.
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
    if timer:
        timer.start_phase("approach")

    # Ensure claw is open before approaching
    send_claw(ser, SCAN_POSE, CLAW_OPEN_POS, label="Ensuring claw is OPEN")

    # Ensure fast profile is active for the approach
    send_raw_command(ser, {"cmd": "set_profile", "vel": DEFAULT_PROFILE_VEL, "acc": DEFAULT_PROFILE_ACC})

    # Compute distance-adjusted grab height so the claw doesn't scrape
    # the desk at far reaches (see config/arm.py for tuning constants)
    grab_z = compute_grab_height(obj_x, obj_y)

    # Compute wrist correction learned during touch calibration
    m4_correction = compute_wrist_correction(obj_x, obj_y)

    # Determine whether touch calibration data exists — if so, grab_z
    # already accounts for real-world sag and the IK solver should NOT
    # apply its own sag compensation (avoids double-compensation).
    has_touch_cal = load_height_calibration() is not None

    pickup_pose = {
        "x": obj_x,
        "y": obj_y,
        "z": grab_z,
        "m4_offset": m4_correction,
        "m5": CLAW_OPEN_POS,
        "skip_sag": has_touch_cal,
    }
    try:
        transport_plan = prevalidate_transport_plan(arm, colour, pickup_pose)
    except (RouteCalibrationError, ValueError) as exc:
        logger.error("Production transport route prevalidation failed closed for %s ball: %s", colour.upper(), exc)
        return False
    plan_by_name = {name: solution for name, solution in transport_plan}
    destination_steps = [(name, solution) for name, solution in transport_plan if name not in {"pickup", "pickup_recovery_clearance"}]

    # Single full move — descend to grab height in one motion
    last_ik = send_strict_solution(
        ser,
        plan_by_name["pickup"],
        label="Full approach to grab position",
        viz=viz,
    )

    # ── 3. GRABBING (adaptive grip) ──────────────────────────────────
    log_state(State.GRABBING, f"Closing claw on {colour.upper()} ball")
    if timer:
        timer.start_phase("grab")

    print(f"\n  🎯  TARGET REACH: {obj_x:.1f} cm, TARGET Z: {grab_z:.1f} cm (distance-adjusted)")
    # Removed: blocking input() halts headless deployment
    # input("  ⏸️   Press ENTER to close claw (check accuracy now)...")

    # Adaptive grip — gradually close until resistance detected
    grip_ok = adaptive_grip(ser, last_ik)

    # ── 3b. VERIFY_GRIP ──────────────────────────────────────────────
    log_state(State.VERIFY_GRIP, "Checking grip instantly")

    # Verify grip only as fallback if adaptive_grip didn't confirm grip
    if not grip_ok:
        print("  🔍  Adaptive grip inconclusive — verifying with verify_grip()...")
        grip_ok = verify_grip(ser, CLAW_CLOSED_POS)

    if not grip_ok:
        logger.warning("PICK FAILED — grip verification failed for %s ball", colour.upper())
        # Restore normal current limit before opening to ensure it can overcome any jams
        send_raw_command(ser, {"cmd": "set_current_limit", "id": 5, "value": M5_DEFAULT_CURRENT_LIMIT})
        # Open claw to release any partial grip
        send_claw(ser, last_ik, CLAW_OPEN_POS, label="OPEN grip (pick failed recovery)")
        
        # Slow down profile for a smooth sweep back to SCAN_POSE
        send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})
        
        # Retreat only through the already-prevalidated pickup recovery waypoint.
        send_strict_solution(
            ser,
            plan_by_name["pickup_recovery_clearance"],
            label="Retreating via prevalidated pickup clearance after failed pick",
            viz=viz,
            settle_time=0.5
        )
        return False

    print(f"  ✅  Grip verified — proceeding with {colour.upper()} ball")

    # Slow down profile for a smooth, continuous sweep back to SCAN_POSE
    send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})

    # Wait only 0.5s so the arm blends the lift and the SCAN_POSE moves
    # into a single, smooth arc.
    # ── 4. SORTING — route-driven rear placement ──────────────────────
    log_state(State.SORTING, f"Moving {colour.upper()} ball through prevalidated rear-placement route")
    if timer:
        timer.start_phase("sort")

    for idx, (name, solution) in enumerate(destination_steps):
        is_last = idx == len(destination_steps) - 1
        last_ik = send_strict_solution(
            ser,
            solution,
            label=f"Route waypoint {name}",
            viz=viz,
            settle_time=0.5 if not is_last else None,
        )

    # ── 5. DROPPING — release at the bin ─────────────────────────────
    log_state(State.DROPPING, f"Releasing {colour.upper()} ball at validated rear drop pose")
    if timer:
        timer.start_phase("drop")
    print(f"  📤  [CLAW] Opening... (dwell {RELEASE_DWELL}s)")
    # Restore normal current limit for opening and for the next cycle
    send_raw_command(ser, {"cmd": "set_current_limit", "id": 5, "value": M5_DEFAULT_CURRENT_LIMIT})
    send_claw(ser, last_ik, CLAW_OPEN_POS, label="OPEN grip (release at rear drop pose)")
    time.sleep(RELEASE_DWELL)
    print(f"  📤  [CLAW] {colour.upper()} ball released at validated rear drop pose")

    log_state(State.DONE, "Cycle complete ✅")
    return True


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
    from simulation.visualizer import ArmVisualizer

    viz = ArmVisualizer()
    print("[INIT] 3-D visualiser ready")

    # ── Open serial connection ────────────────────────────────────────
    if USE_REAL_SERIAL:
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
            logger.warning("[INIT] Did not receive 'OK:READY' from OpenRB-150!")
            print("       Check: (1) firmware uploaded? (2) correct port? (3) baud rate?")
    else:
        ser = MockSerial(move_delay=1.0, visualizer=viz, anim_frames=30)

    # ── Initialise vision bridge ──────────────────────────────────────
    vision = VisionBridge(use_camera=USE_REAL_CAMERA)
    if not vision.open():
        logger.error("[INIT] Vision bridge failed to open real camera.")
        logger.error("       Check: (1) USB connection? (2) Re-plug the camera? (3) Power?")
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
    timer = CycleTimer()
    pick_fail_count = 0         # consecutive pick failures for retry logic

    try:
        while True:
            scan_round += 1

            # ── MOVE_TO_SCAN_POSE ─────────────────────────────────────
            log_state(State.MOVE_TO_SCAN_POSE, f"Preparing for scan round {scan_round}")
            send_scan_pose(ser, viz=viz)

            # ── SCANNING ──────────────────────────────────────────────
            log_state(State.SCANNING, f"Scan round {scan_round}")
            time.sleep(SCAN_INTERVAL)   # wait SCAN_INTERVAL seconds before capturing frame
            verify_scan_pose_before_scan(ser, vision)

            timer.start_cycle()
            timer.start_phase("scan")
            detections = vision.scan_for_balls()

            if not detections:
                # No balls found — wait before rescanning.
                # Reset round counter so it never "runs out".
                scan_round = 0
                pick_fail_count = 0     # reset on new scan cycle
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

            success = run_sorting_cycle(ser, arm, detection, vision, viz=viz, timer=timer)

            if success:
                timings = timer.end_cycle()
                timer.print_cycle_summary(timings)
                pick_fail_count = 0     # reset on successful pick
                print("\n[DONE] Object processed — returning to SCAN_POSE for next cycle...")
            else:
                pick_fail_count += 1
                print(f"\n[PICK FAILED] Attempt {pick_fail_count}/{MAX_PICK_RETRIES} "
                      f"for this detection")
                if pick_fail_count >= MAX_PICK_RETRIES:
                    logger.warning("Skipping unreachable ball after %d failed attempts", MAX_PICK_RETRIES)
                    pick_fail_count = 0
                else:
                    print("  🔄  Returning to SCAN_POSE for retry (no idle delay)...")
                # Skip idle delay — immediately loop back to scan
                continue

            # In simulation mode, one pass is enough (fake data won't change)
            if not USE_REAL_CAMERA:
                print("[QUEUE] Simulation mode — exiting after one pass")
                break

    except KeyboardInterrupt:
        print(f"\n\n{'━' * 60}")
        print("  ⛔  KeyboardInterrupt received — shutting down gracefully...")
        print(f"{'━' * 60}")

    # Print session timing summary
    timer.print_session_summary()

    # Return arm to SCAN_POSE before powering off
    try:
        log_state(State.IDLE, "Returning to SCAN_POSE before shutdown")
        send_scan_pose(ser, viz=viz)
    except Exception as e:
        logger.warning("Could not return to SCAN_POSE: %s", e)

    # ── Shutdown ──────────────────────────────────────────────────────
    print(f"\n{'━' * 60}")
    print("  🏁  Arm is at SCAN_POSE.  Shutting down.")
    print(f"{'━' * 60}\n")

    vision.close()                # releases camera + destroys OpenCV windows
    cv2.destroyAllWindows()       # safety fallback
    ser.close()
    viz.close()                   # blocks until user closes the plot window


if __name__ == "__main__":
    main()
