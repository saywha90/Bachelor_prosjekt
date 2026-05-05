"""
Interactive rear-bin route fine-tuning for real hardware calibration.

This tool edits the production route schema in ``bin_calibration.json`` for the
two physical rear bins only: ``RED_BIN`` and ``BLUE_BIN``.  It intentionally does
not create or save ``REJECT_BIN`` because production should return to scanning
when no object is gripped instead of routing to a reject bin.

The default mode connects to the arm, mirrors the touch-calibration workflow,
and supports explicit hardware movement plus limp-mode pose capture.  Offline
editing and test runs are still available with ``--dry-run``/``--no-hardware``.

Usage
-----
    PYTHONPATH=src python3 src/calibration/10_bin_calibration.py
    PYTHONPATH=src python3 src/calibration/10_bin_calibration.py --port /dev/cu.usbmodem101
    PYTHONPATH=src python3 src/calibration/10_bin_calibration.py --dry-run
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import date, datetime
from pathlib import Path
from typing import Any, Callable

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from config.arm import (  # noqa: E402
    BIN_CALIBRATION_FILE as CONFIG_BIN_CALIBRATION_FILE,
    DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG,
    SCAN_POSE,
    RouteCalibrationError,
    load_transport_route_calibration,
)
from ik.solver import ArmIK  # noqa: E402


BIN_CALIBRATION_FILE = Path(CONFIG_BIN_CALIBRATION_FILE)
SERIAL_PORT = "/dev/cu.usbmodem101"
SERIAL_BAUD = 115200
MOVE_SETTLE_SECONDS = 0.75
ROUTE_SETTLE_SECONDS = 0.75
LIMP_MOTOR_IDS = (1, 2, 3, 4)
MAX_CAPTURE_M4_OFFSET = 1500

ROUTE_SCHEMA_VERSION = 2
ONLY_BIN_NAMES = ("RED_BIN", "BLUE_BIN")
SHARED_WAYPOINTS = ("front_neutral", "rear_transfer")
BIN_POSES = ("approach", "drop")
POSE_FIELDS = ("x", "y", "z", "m4_offset")


@dataclass(frozen=True)
class WaypointSpec:
    """Editable route waypoint descriptor."""

    key: str
    title: str
    intent: str
    root: str
    bin_name: str | None = None
    pose_name: str | None = None


EDITABLE_WAYPOINTS = (
    WaypointSpec("front_neutral", "front_neutral", "carry", "shared"),
    WaypointSpec("rear_transfer", "rear_transfer", "rear_place", "shared"),
    WaypointSpec("RED_BIN.approach", "RED_BIN.approach", "rear_place", "bin", "RED_BIN", "approach"),
    WaypointSpec("RED_BIN.drop", "RED_BIN.drop", "rear_place", "bin", "RED_BIN", "drop"),
    WaypointSpec("BLUE_BIN.approach", "BLUE_BIN.approach", "rear_place", "bin", "BLUE_BIN", "approach"),
    WaypointSpec("BLUE_BIN.drop", "BLUE_BIN.drop", "rear_place", "bin", "BLUE_BIN", "drop"),
)


def _safe_default_route_schema() -> dict[str, Any]:
    """Return a conservative valid two-bin rear route schema."""

    return {
        "schema_version": ROUTE_SCHEMA_VERSION,
        "calibration_date": date.today().isoformat(),
        "calibrated_with_scan_pose": {k: int(v) for k, v in SCAN_POSE.items()},
        "rear_base_yaw_limit_deg": float(DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG),
        "shared_waypoints": {
            "front_neutral": {
                "x": 22.0,
                "y": 0.0,
                "z": 30.0,
                "m4_offset": 0,
                "skip_sag": True,
            },
            "rear_transfer": {
                "x": -22.0,
                "y": 0.0,
                "z": 38.0,
                "m4_offset": 0,
                "skip_sag": True,
            },
        },
        "bins": {
            "RED_BIN": {
                "approach": {
                    "x": -24.0,
                    "y": -7.0,
                    "z": 38.0,
                    "m4_offset": 0,
                    "skip_sag": True,
                },
                "drop": {
                    "x": -24.0,
                    "y": -7.0,
                    "z": 33.0,
                    "m4_offset": 0,
                    "skip_sag": True,
                },
            },
            "BLUE_BIN": {
                "approach": {
                    "x": -24.0,
                    "y": 7.0,
                    "z": 38.0,
                    "m4_offset": 0,
                    "skip_sag": True,
                },
                "drop": {
                    "x": -24.0,
                    "y": 7.0,
                    "z": 33.0,
                    "m4_offset": 0,
                    "skip_sag": True,
                },
            },
        },
    }


def _route_calibration_to_schema(route_calibration: Any) -> dict[str, Any]:
    """Convert a validated config.arm transport route to writable JSON data."""

    def pose_to_dict(pose: Any) -> dict[str, Any]:
        return {
            "x": round(float(pose.x), 2),
            "y": round(float(pose.y), 2),
            "z": round(float(pose.z), 2),
            "m4_offset": int(pose.m4_offset),
            "skip_sag": bool(pose.skip_sag),
        }

    schema = _safe_default_route_schema()
    schema["schema_version"] = max(int(route_calibration.schema_version), ROUTE_SCHEMA_VERSION)
    schema["rear_base_yaw_limit_deg"] = float(
        getattr(
            route_calibration,
            "rear_base_yaw_limit_deg",
            DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG,
        )
    )
    for waypoint in SHARED_WAYPOINTS:
        schema["shared_waypoints"][waypoint] = pose_to_dict(route_calibration.shared_waypoints[waypoint])
    for bin_name in ONLY_BIN_NAMES:
        bin_route = route_calibration.bins[bin_name]
        schema["bins"][bin_name] = {
            "approach": pose_to_dict(bin_route.approach),
            "drop": pose_to_dict(bin_route.drop),
        }
    return schema


def load_or_initialize_route_schema(path: Path = BIN_CALIBRATION_FILE) -> tuple[dict[str, Any], list[str]]:
    """Load current route schema or return a safe two-bin initialization.

    The returned schema is always constrained to ``RED_BIN`` and ``BLUE_BIN``.
    Legacy, missing, incomplete, invalid, or extra-bin files never propagate a
    ``REJECT_BIN`` entry into the editable data.
    """

    messages: list[str] = []
    try:
        route = load_transport_route_calibration(path=path, require_route_schema=True)
        missing_bins = [bin_name for bin_name in ONLY_BIN_NAMES if bin_name not in route.bins]
        if missing_bins:
            raise RouteCalibrationError(f"route schema missing required bin(s): {', '.join(missing_bins)}")
        schema = _route_calibration_to_schema(route)
        messages.append(f"Loaded route schema from {path}")
        if any(bin_name not in ONLY_BIN_NAMES for bin_name in route.bins):
            messages.append("Ignored non-production bin entries while loading; only RED_BIN and BLUE_BIN are editable")
        return schema, messages
    except RouteCalibrationError as exc:
        messages.append(f"Using safe two-bin route defaults because calibration could not be loaded: {exc}")
    except FileNotFoundError as exc:
        messages.append(f"Using safe two-bin route defaults because calibration file is missing: {exc}")

    return _safe_default_route_schema(), messages


def _save_json_atomic(path: Path, data: dict[str, Any]) -> None:
    """Write JSON atomically by replacing with a fully-written temp file."""

    path = Path(path)
    tmp_fd, tmp_path = tempfile.mkstemp(dir=str(path.parent), suffix=".tmp")
    try:
        with os.fdopen(tmp_fd, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
        os.replace(tmp_path, path)
    except BaseException:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        raise


def save_route_schema_with_backup(data: dict[str, Any], path: Path = BIN_CALIBRATION_FILE) -> Path | None:
    """Save route schema, creating a timestamped backup before overwriting."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    backup_path: Path | None = None
    if path.exists():
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = path.with_name(f"{path.stem}.backup_{timestamp}{path.suffix}")
        shutil.copy2(path, backup_path)
    data_to_save = sanitize_two_bin_schema(data)
    data_to_save["calibration_date"] = date.today().isoformat()
    data_to_save["calibrated_with_scan_pose"] = {k: int(v) for k, v in SCAN_POSE.items()}
    _save_json_atomic(path, data_to_save)
    return backup_path


def sanitize_two_bin_schema(data: dict[str, Any]) -> dict[str, Any]:
    """Return a route schema containing only supported two-bin route data."""

    defaults = _safe_default_route_schema()
    sanitized = copy.deepcopy(defaults)
    sanitized["schema_version"] = int(data.get("schema_version", data.get("version", ROUTE_SCHEMA_VERSION)) or ROUTE_SCHEMA_VERSION)
    sanitized["schema_version"] = max(sanitized["schema_version"], ROUTE_SCHEMA_VERSION)
    sanitized["rear_base_yaw_limit_deg"] = _parse_rear_base_yaw_limit(
        data.get("rear_base_yaw_limit_deg", DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG)
    )

    shared_raw = data.get("shared_waypoints") if isinstance(data.get("shared_waypoints"), dict) else {}
    for waypoint in SHARED_WAYPOINTS:
        if isinstance(shared_raw.get(waypoint), dict):
            sanitized["shared_waypoints"][waypoint] = _sanitize_pose(shared_raw[waypoint], defaults["shared_waypoints"][waypoint])

    bins_raw = data.get("bins") if isinstance(data.get("bins"), dict) else {}
    for bin_name in ONLY_BIN_NAMES:
        raw_bin = bins_raw.get(bin_name) if isinstance(bins_raw.get(bin_name), dict) else {}
        for pose_name in BIN_POSES:
            if isinstance(raw_bin.get(pose_name), dict):
                sanitized["bins"][bin_name][pose_name] = _sanitize_pose(
                    raw_bin[pose_name], defaults["bins"][bin_name][pose_name]
                )
    return sanitized


def _parse_rear_base_yaw_limit(raw: Any) -> float:
    try:
        limit = abs(float(raw))
    except (TypeError, ValueError):
        return float(DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG)
    if not 0.0 <= limit <= 180.0:
        return float(DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG)
    return limit


def _sanitize_pose(raw_pose: dict[str, Any], default_pose: dict[str, Any]) -> dict[str, Any]:
    pose = copy.deepcopy(default_pose)
    for field in ("x", "y", "z"):
        try:
            pose[field] = round(float(raw_pose.get(field, pose[field])), 2)
        except (TypeError, ValueError):
            pass
    try:
        pose["m4_offset"] = int(raw_pose.get("m4_offset", pose.get("m4_offset", 0)) or 0)
    except (TypeError, ValueError):
        pose["m4_offset"] = int(default_pose.get("m4_offset", 0) or 0)
    pose["skip_sag"] = bool(raw_pose.get("skip_sag", pose.get("skip_sag", True)))
    return pose


def get_waypoint(data: dict[str, Any], spec: WaypointSpec) -> dict[str, Any]:
    """Return the mutable waypoint pose dictionary for a waypoint spec."""

    if spec.root == "shared":
        return data["shared_waypoints"][spec.key]
    if spec.bin_name is None or spec.pose_name is None:
        raise ValueError(f"Invalid bin waypoint spec: {spec}")
    return data["bins"][spec.bin_name][spec.pose_name]


def update_waypoint_field(
    data: dict[str, Any],
    spec: WaypointSpec,
    field: str,
    delta: float,
    *,
    clamp_m4: bool = True,
) -> dict[str, Any]:
    """Apply a small field adjustment to a waypoint and return that waypoint."""

    if field not in POSE_FIELDS:
        raise ValueError(f"Unsupported waypoint field {field!r}; expected one of {POSE_FIELDS}")
    waypoint = get_waypoint(data, spec)
    if field == "m4_offset":
        next_value = int(waypoint.get(field, 0)) + int(round(delta))
        if clamp_m4:
            next_value = max(-1500, min(1500, next_value))
        waypoint[field] = next_value
    else:
        waypoint[field] = round(float(waypoint.get(field, 0.0)) + float(delta), 2)
    return waypoint


def strict_pose_for_validation(data: dict[str, Any], spec: WaypointSpec) -> dict[str, Any]:
    """Build the pose mapping passed to ArmIK.solve_strict()."""

    pose = dict(get_waypoint(data, spec))
    pose.setdefault("m4_offset", 0)
    pose.setdefault("skip_sag", True)
    if spec.intent == "rear_place":
        pose["rear_base_yaw_limit_deg"] = data.get(
            "rear_base_yaw_limit_deg",
            DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG,
        )
    return pose


def validate_waypoint(data: dict[str, Any], spec: WaypointSpec, arm: ArmIK | None = None) -> dict[str, Any]:
    """Validate a waypoint with ArmIK.solve_strict() and return a result dict."""

    if arm is None:
        arm = ArmIK(rear_base_yaw_limit_deg=data.get("rear_base_yaw_limit_deg", DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG))
    pose = strict_pose_for_validation(data, spec)
    try:
        solved = arm.solve_strict(pose, intent=spec.intent)
        validation = solved["validation"]
        return {
            "ok": True,
            "reason": "OK",
            "commands": solved["commands"],
            "base_yaw_deg": float(validation["base_yaw_deg"]),
            "branch": validation["ik_branch"],
            "shoulder_deg": float(validation["theta_shoulder_deg"]),
            "validation": validation,
        }
    except ValueError as exc:
        return {
            "ok": False,
            "reason": str(exc),
            "commands": None,
            "base_yaw_deg": None,
            "branch": None,
            "shoulder_deg": None,
            "validation": None,
        }


def validate_all_waypoints(data: dict[str, Any], arm: ArmIK | None = None) -> dict[str, dict[str, Any]]:
    """Validate all editable waypoints."""

    return {spec.key: validate_waypoint(data, spec, arm) for spec in EDITABLE_WAYPOINTS}


def print_validation_result(spec: WaypointSpec, result: dict[str, Any]) -> None:
    """Print concise validation details for one waypoint."""

    if result["ok"]:
        print(f"  ✅ {spec.title}: OK")
        print(f"     base_yaw={result['base_yaw_deg']:.2f}°  branch={result['branch']}  shoulder={result['shoulder_deg']:.2f}°")
        print(f"     commands={json.dumps(result['commands'], sort_keys=True)}")
    else:
        print(f"  ❌ {spec.title}: {result['reason']}")


def print_route_summary(data: dict[str, Any], selected_index: int | None = None) -> None:
    """Show current editable route values."""

    print("\nCurrent rear-bin route calibration")
    print(f"  rear_base_yaw_limit_deg = {float(data.get('rear_base_yaw_limit_deg', DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG)):.1f}")
    for idx, spec in enumerate(EDITABLE_WAYPOINTS, start=1):
        pose = get_waypoint(data, spec)
        marker = "→" if selected_index == idx - 1 else " "
        print(
            f"{marker} [{idx}] {spec.title:<17} "
            f"x={float(pose['x']):>7.2f}  y={float(pose['y']):>7.2f}  "
            f"z={float(pose['z']):>7.2f}  m4_offset={int(pose.get('m4_offset', 0)):>+5d}  "
            f"skip_sag={bool(pose.get('skip_sag', True))}"
        )


def _open_serial(port: str, baud: int):
    """Open serial connection to the OpenRB bridge."""

    import serial

    print(f"[SERIAL] Opening {port} @ {baud} …")
    ser = serial.Serial(port, baud, timeout=2)
    time.sleep(3)
    boot_msg = ""
    while ser.in_waiting:
        boot_msg += ser.readline().decode(errors="replace").strip() + " "
    if not boot_msg.strip():
        boot_msg = ser.readline().decode(errors="replace").strip()
    print(f"[SERIAL] OpenRB says: {boot_msg.strip()}")
    send_raw_command(ser, {"cmd": "enable_torque"})
    send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})
    return ser


def send_raw_command(ser: Any, command: dict[str, Any]) -> str:
    """Send one JSON command to hardware and return the firmware response."""

    ser.write((json.dumps(command) + "\n").encode())
    return ser.readline().decode(errors="replace").strip()


def read_motor_positions(ser: Any) -> dict[str, int] | None:
    """Read current motor positions from the firmware."""

    response = send_raw_command(ser, {"cmd": "read_pos"})
    try:
        positions = json.loads(response)
    except json.JSONDecodeError:
        print(f"  ❌ Invalid read_pos response: {response}")
        return None
    if not isinstance(positions, dict):
        print(f"  ❌ Unexpected read_pos response: {positions!r}")
        return None
    missing = {"m1", "m2", "m3", "m4"} - set(positions)
    if missing:
        print(f"  ❌ read_pos response missing motor keys: {sorted(missing)}")
        return None
    try:
        return {key: int(value) for key, value in positions.items() if str(key).startswith("m")}
    except (TypeError, ValueError):
        print(f"  ❌ Non-integer motor position in read_pos response: {positions}")
        return None


def _set_current_goals_before_torque_on(ser: Any, positions: dict[str, int] | None = None) -> None:
    """Best-effort snap-back prevention before enabling torque."""

    if positions is None:
        positions = read_motor_positions(ser)
    if positions is not None:
        print(f"  [SAFETY] Setting goal positions to current positions before torque-on: {positions}")
        send_raw_command(ser, positions)


def disable_limp_mode(ser: Any, input_func: Callable[[str], str] = input) -> bool:
    """Disable torque on arm motors 1-4 so the arm can be guided by hand."""

    print("\n🖐️  Limp mode requested.")
    print("  ⚠️  SUPPORT THE ARM before disabling torque; it can fall under gravity.")
    confirm = input_func("  Type LIMP when you are supporting the arm, or anything else to cancel: ").strip()
    if confirm != "LIMP":
        print("  Limp mode cancelled; torque was not changed.")
        return False
    for motor_id in LIMP_MOTOR_IDS:
        response = send_raw_command(ser, {"cmd": "set_torque", "id": motor_id, "enable": False})
        if "ERR" in response.upper():
            print(f"  ⚠️  Motor {motor_id} torque disable warning: {response}")
    print("  🔓 Motors 1-4 are now limp. Guide the arm by hand; use 'capture' or 'lock' when done.")
    return True


def enable_limp_mode(ser: Any) -> bool:
    """Re-enable torque safely by first setting goals to current positions."""

    print("\n🔒 Re-enabling torque safely.")
    try:
        positions = read_motor_positions(ser)
        _set_current_goals_before_torque_on(ser, positions)
        response = send_raw_command(ser, {"cmd": "enable_torque"})
        print(f"  Torque response: {response}")
        print("  ✅ Torque re-enabled; arm should hold its current pose.")
        return True
    except Exception as exc:
        print(f"  ⚠️  Failed to re-enable torque: {exc}")
        print("  ⚠️  Manually support the arm and power-cycle if needed.")
        return False


def capture_current_waypoint(
    data: dict[str, Any],
    spec: WaypointSpec,
    ser: Any,
    arm: ArmIK,
) -> bool:
    """Capture current motor pose with FK and store it in the selected waypoint."""

    positions = read_motor_positions(ser)
    if positions is None:
        print("  Capture failed: current motor positions could not be read.")
        return False

    try:
        fk = arm.forward_kinematics(positions)
    except Exception as exc:
        print(f"  Capture failed: forward kinematics could not convert current motors: {exc}")
        enable_limp_mode(ser)
        return False

    old_pose = copy.deepcopy(get_waypoint(data, spec))
    candidate = copy.deepcopy(old_pose)
    candidate["x"] = round(float(fk["x"]), 2)
    candidate["y"] = round(float(fk["y"]), 2)
    candidate["z"] = round(float(fk["z"]), 2)

    # Compute the saved trim as the final claw pitch's deviation from IK's
    # default straight-down pitch.  ``m4_offset`` participates in IK geometry,
    # so FK-captured limp poses can replay the same XYZ instead of rotating the
    # L3 claw-tip link after the Cartesian solve.
    try:
        raw_offset = int(fk.get("replay_m4_offset", 0))
        candidate["m4_offset"] = max(
            -MAX_CAPTURE_M4_OFFSET,
            min(MAX_CAPTURE_M4_OFFSET, raw_offset),
        )
        if candidate["m4_offset"] != raw_offset:
            print(
                f"  ⚠️  Wrist replay offset clipped from {raw_offset:+d} "
                f"to {candidate['m4_offset']:+d} steps for safety."
            )
    except Exception as exc:
        candidate["m4_offset"] = int(old_pose.get("m4_offset", 0) or 0)
        print(f"  ⚠️  Could not derive wrist trim from strict IK ({exc}); keeping previous m4_offset={candidate['m4_offset']}.")

    get_waypoint(data, spec).clear()
    get_waypoint(data, spec).update(candidate)
    result = validate_waypoint(data, spec, arm)
    print("\nCaptured current pose:")
    print(f"  motors={json.dumps(positions, sort_keys=True)}")
    print(
        f"  fk={{x={candidate['x']:.2f}, y={candidate['y']:.2f}, z={candidate['z']:.2f}, "
        f"pitch={float(fk.get('theta_pitch_deg', 0.0)):+.1f}°, replay_m4_offset={candidate['m4_offset']:+d}}}"
    )
    print_validation_result(spec, result)

    if not result["ok"]:
        get_waypoint(data, spec).clear()
        get_waypoint(data, spec).update(old_pose)
        print("  Capture rejected and reverted because strict IK validation failed.")
        enable_limp_mode(ser)
        return False

    enable_limp_mode(ser)
    print("  ✅ Captured pose stored in memory. Use 'save' to persist it.")
    return True


def send_validated_commands(ser: Any, commands: dict[str, int], *, label: str, settle_seconds: float = MOVE_SETTLE_SECONDS) -> str:
    """Send prevalidated motor commands and wait briefly for motion to settle."""

    print(f"  📍 {label}: commands={json.dumps(commands, sort_keys=True)}")
    response = send_raw_command(ser, commands)
    print(f"     Firmware response: {response}")
    if settle_seconds > 0:
        time.sleep(settle_seconds)
    return response


def maybe_move_hardware(ser: Any, commands: dict[str, int], input_func: Callable[[str], str] = input) -> None:
    """Explicitly confirm before moving the arm to validated motor commands."""

    print("\nHardware move requested.")
    print(f"  Commands: {json.dumps(commands, sort_keys=True)}")
    confirm = input_func("  Type MOVE to send these motor goals, or anything else to cancel: ").strip()
    if confirm != "MOVE":
        print("  Move cancelled; no hardware command sent.")
        return
    send_validated_commands(ser, commands, label="Selected waypoint")


def route_specs_for_bin(bin_name: str) -> list[WaypointSpec]:
    """Return editable waypoint specs in movement order for one rear bin."""

    normalized = bin_name.upper()
    if normalized in {"RED", "RED_BIN"}:
        target = "RED_BIN"
    elif normalized in {"BLUE", "BLUE_BIN"}:
        target = "BLUE_BIN"
    else:
        raise ValueError("expected RED_BIN or BLUE_BIN")
    return [
        EDITABLE_WAYPOINTS[0],
        EDITABLE_WAYPOINTS[1],
        next(spec for spec in EDITABLE_WAYPOINTS if spec.key == f"{target}.approach"),
        next(spec for spec in EDITABLE_WAYPOINTS if spec.key == f"{target}.drop"),
    ]


def route_specs_for_selected(spec: WaypointSpec) -> list[WaypointSpec]:
    """Return selected bin route specs, or raise for shared-only selections."""

    if spec.bin_name is None:
        raise ValueError("selected waypoint is shared; use 'test red' or 'test blue'")
    return route_specs_for_bin(spec.bin_name)


def maybe_test_route_hardware(
    data: dict[str, Any],
    *,
    ser: Any,
    arm: ArmIK,
    specs: list[WaypointSpec],
    label: str,
    confirmation: str,
    input_func: Callable[[str], str] = input,
) -> bool:
    """Validate, display, confirm, and move through multiple route waypoints."""

    print(f"\nHardware route test requested: {label}")
    route_commands: list[tuple[WaypointSpec, dict[str, Any]]] = []
    all_ok = True
    for spec in specs:
        result = validate_waypoint(data, spec, arm)
        print_validation_result(spec, result)
        all_ok = all_ok and result["ok"]
        if result["ok"]:
            route_commands.append((spec, result["commands"]))
    if not all_ok:
        print("  Route test refused because one or more waypoints failed strict validation.")
        return False

    print("\n  Movement order:")
    for idx, (spec, commands) in enumerate(route_commands, start=1):
        print(f"    {idx}. {spec.title}: {json.dumps(commands, sort_keys=True)}")
    confirm = input_func(f"  Type {confirmation} to move through these {len(route_commands)} waypoints: ").strip()
    if confirm != confirmation:
        print("  Route test cancelled; no hardware command sent.")
        return False

    send_raw_command(ser, {"cmd": "set_profile", "vel": 40, "acc": 10})
    for spec, commands in route_commands:
        send_validated_commands(ser, commands, label=spec.title, settle_seconds=ROUTE_SETTLE_SECONDS)
    print("  ✅ Route test complete.")
    return True


def interactive_loop(
    data: dict[str, Any],
    *,
    hardware: bool = False,
    ser: Any = None,
    input_func: Callable[[str], str] = input,
) -> bool:
    """Run the terminal interactive route editor.  Returns True if saved."""

    selected_index = 0
    xyz_step = 0.5
    m4_step = 25
    saved = False
    dirty = False

    print("\nInteractive controls")
    print("  select 1-6     choose waypoint")
    print("  x+/x- y+/y- z+/z-  adjust Cartesian fields by current cm step")
    print("  m4+/m4-        adjust wrist trim by current motor-step size")
    print("  step <cm>      set Cartesian step size, e.g. step 0.1")
    print("  m4step <n>     set m4_offset step size, e.g. m4step 10")
    print("  yaw <deg>      set rear base yaw limit, default 45")
    print("  v              validate selected waypoint")
    print("  va             validate all waypoints")
    print("  move           validate and move to selected waypoint after MOVE confirmation")
    print("  test red/blue  validate and move through one bin route after TEST RED/BLUE confirmation")
    print("  test selected  validate and move through the selected bin route")
    print("  test all       validate and move through RED then BLUE routes after TEST ALL confirmation")
    print("  limp           hardware only: disable torque on motors 1-4 for hand-guiding")
    print("  lock           hardware only: set current goals and re-enable torque")
    print("  capture        hardware only: read current motors, FK-convert, store selected waypoint, and lock")
    print("  pos            hardware only: read current motor positions")
    print("  save           validate all, confirm, backup, and save")
    print("  q              quit without saving")
    print("\nNote: production route schema supports x/y/z/m4_offset/skip_sag here; m5/claw is not saved by config.arm routes.")

    arm = ArmIK(rear_base_yaw_limit_deg=data.get("rear_base_yaw_limit_deg", DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG))
    last_valid_commands: dict[str, int] | None = None
    last_valid_key: str | None = None

    while True:
        spec = EDITABLE_WAYPOINTS[selected_index]
        print_route_summary(data, selected_index)
        print(f"\nSelected: {spec.title} | xyz_step={xyz_step:.2f} cm | m4_step={m4_step} steps")
        command = input_func("bin-route> ").strip()
        if not command:
            continue
        lowered = command.lower()

        if lowered in {"q", "quit", "exit"}:
            if dirty:
                confirm = input_func("Unsaved edits exist. Type DISCARD to quit without saving: ").strip()
                if confirm != "DISCARD":
                    continue
            print("Exiting without additional saves.")
            return saved

        if lowered in {"h", "help", "?"}:
            print("Commands: 1-6, x+/x-, y+/y-, z+/z-, m4+/m4-, step <cm>, m4step <n>, yaw <deg>, v, va, move, test red/blue/selected/all, limp, lock, capture, pos, save, q")
            continue

        if lowered.isdigit() and 1 <= int(lowered) <= len(EDITABLE_WAYPOINTS):
            selected_index = int(lowered) - 1
            last_valid_commands = None
            last_valid_key = None
            continue

        if lowered.startswith("step "):
            try:
                xyz_step = max(0.01, min(5.0, abs(float(lowered.split(maxsplit=1)[1]))))
                print(f"  Cartesian step set to {xyz_step:.2f} cm")
            except ValueError:
                print("  Invalid step value")
            continue

        if lowered.startswith("m4step "):
            try:
                m4_step = max(1, min(250, abs(int(lowered.split(maxsplit=1)[1]))))
                print(f"  m4_offset step set to {m4_step} motor steps")
            except ValueError:
                print("  Invalid m4step value")
            continue

        if lowered.startswith("yaw "):
            try:
                data["rear_base_yaw_limit_deg"] = _parse_rear_base_yaw_limit(lowered.split(maxsplit=1)[1])
                arm = ArmIK(rear_base_yaw_limit_deg=data["rear_base_yaw_limit_deg"])
                dirty = True
                last_valid_commands = None
                last_valid_key = None
                print(f"  rear_base_yaw_limit_deg set to {data['rear_base_yaw_limit_deg']:.1f}")
            except ValueError:
                print("  Invalid yaw value")
            continue

        field_delta: tuple[str, float] | None = None
        if lowered in {"x+", "xp"}:
            field_delta = ("x", xyz_step)
        elif lowered in {"x-", "xm"}:
            field_delta = ("x", -xyz_step)
        elif lowered in {"y+", "yp"}:
            field_delta = ("y", xyz_step)
        elif lowered in {"y-", "ym"}:
            field_delta = ("y", -xyz_step)
        elif lowered in {"z+", "zp"}:
            field_delta = ("z", xyz_step)
        elif lowered in {"z-", "zm"}:
            field_delta = ("z", -xyz_step)
        elif lowered in {"m4+", "m4p", "i"}:
            field_delta = ("m4_offset", m4_step)
        elif lowered in {"m4-", "m4m", "k"}:
            field_delta = ("m4_offset", -m4_step)

        if field_delta is not None:
            field, delta = field_delta
            old_pose = copy.deepcopy(get_waypoint(data, spec))
            update_waypoint_field(data, spec, field, delta)
            result = validate_waypoint(data, spec, arm)
            print_validation_result(spec, result)
            if not result["ok"]:
                get_waypoint(data, spec).clear()
                get_waypoint(data, spec).update(old_pose)
                print("  Edit reverted because strict IK validation failed.")
                last_valid_commands = None
                last_valid_key = None
            else:
                dirty = True
                last_valid_commands = result["commands"]
                last_valid_key = spec.key
                print("  Edit kept in memory. Use 'move' to test on hardware and 'save' to persist it; hardware did not move automatically.")
            continue

        if lowered == "v":
            result = validate_waypoint(data, spec, arm)
            print_validation_result(spec, result)
            last_valid_commands = result["commands"] if result["ok"] else None
            last_valid_key = spec.key if result["ok"] else None
            continue

        if lowered == "va":
            results = validate_all_waypoints(data, arm)
            for waypoint_spec in EDITABLE_WAYPOINTS:
                print_validation_result(waypoint_spec, results[waypoint_spec.key])
            continue

        if lowered == "move":
            if not hardware:
                print("  Dry-run/no-hardware mode: no serial connection is open. Run without --dry-run to enable moves.")
                continue
            result = validate_waypoint(data, spec, arm)
            print_validation_result(spec, result)
            if not result["ok"]:
                print("  Move refused because selected waypoint is invalid.")
                continue
            last_valid_commands = result["commands"]
            last_valid_key = spec.key
            maybe_move_hardware(ser, last_valid_commands, input_func)
            continue

        if lowered in {"limp", "l"}:
            if not hardware:
                print("  Dry-run/no-hardware mode: limp mode requires a serial connection. Run without --dry-run.")
                continue
            disable_limp_mode(ser, input_func)
            continue

        if lowered in {"lock", "torque", "torque on"}:
            if not hardware:
                print("  Dry-run/no-hardware mode: torque control requires a serial connection. Run without --dry-run.")
                continue
            enable_limp_mode(ser)
            continue

        if lowered in {"capture", "cap", "read capture"}:
            if not hardware:
                print("  Dry-run/no-hardware mode: current-position capture requires a serial connection. Run without --dry-run.")
                continue
            if capture_current_waypoint(data, spec, ser, arm):
                dirty = True
                result = validate_waypoint(data, spec, arm)
                last_valid_commands = result["commands"] if result["ok"] else None
                last_valid_key = spec.key if result["ok"] else None
            continue

        if lowered in {"pos", "read", "read_pos"}:
            if not hardware:
                print("  Dry-run/no-hardware mode: motor readout requires a serial connection. Run without --dry-run.")
                continue
            positions = read_motor_positions(ser)
            if positions is not None:
                print(f"  Current motor positions: {json.dumps(positions, sort_keys=True)}")
            continue

        if lowered.startswith("test"):
            if not hardware:
                print("  Dry-run/no-hardware mode: route movement requires a serial connection. Run without --dry-run.")
                continue
            parts = lowered.split()
            target = parts[1] if len(parts) > 1 else "selected"
            try:
                if target in {"red", "red_bin"}:
                    maybe_test_route_hardware(
                        data,
                        ser=ser,
                        arm=arm,
                        specs=route_specs_for_bin("RED_BIN"),
                        label="RED_BIN route",
                        confirmation="TEST RED",
                        input_func=input_func,
                    )
                elif target in {"blue", "blue_bin"}:
                    maybe_test_route_hardware(
                        data,
                        ser=ser,
                        arm=arm,
                        specs=route_specs_for_bin("BLUE_BIN"),
                        label="BLUE_BIN route",
                        confirmation="TEST BLUE",
                        input_func=input_func,
                    )
                elif target == "selected":
                    selected_route = route_specs_for_selected(spec)
                    selected_bin = spec.bin_name or "selected"
                    maybe_test_route_hardware(
                        data,
                        ser=ser,
                        arm=arm,
                        specs=selected_route,
                        label=f"{selected_bin} route",
                        confirmation=f"TEST {selected_bin.removesuffix('_BIN')}",
                        input_func=input_func,
                    )
                elif target == "all":
                    all_specs = route_specs_for_bin("RED_BIN") + route_specs_for_bin("BLUE_BIN")
                    maybe_test_route_hardware(
                        data,
                        ser=ser,
                        arm=arm,
                        specs=all_specs,
                        label="RED_BIN route followed by BLUE_BIN route",
                        confirmation="TEST ALL",
                        input_func=input_func,
                    )
                else:
                    print("  Unknown test target. Use: test red, test blue, test selected, or test all.")
            except ValueError as exc:
                print(f"  Route test unavailable: {exc}")
            continue

        if lowered == "save":
            results = validate_all_waypoints(data, arm)
            all_ok = True
            for waypoint_spec in EDITABLE_WAYPOINTS:
                result = results[waypoint_spec.key]
                print_validation_result(waypoint_spec, result)
                all_ok = all_ok and result["ok"]
            if not all_ok:
                print("  Save refused: fix invalid waypoints first.")
                continue
            confirm = input_func(f"Type SAVE to overwrite {BIN_CALIBRATION_FILE} with a backed-up two-bin route schema: ").strip()
            if confirm != "SAVE":
                print("  Save cancelled.")
                continue
            backup = save_route_schema_with_backup(data, BIN_CALIBRATION_FILE)
            print(f"  Saved {BIN_CALIBRATION_FILE}")
            if backup is not None:
                print(f"  Backup written to {backup}")
            print("  Confirmed: only RED_BIN and BLUE_BIN were written; REJECT_BIN was not added.")
            dirty = False
            saved = True
            continue

        if lowered == "last" and last_valid_commands is not None:
            print(f"  Last valid commands for {last_valid_key}: {json.dumps(last_valid_commands, sort_keys=True)}")
            continue

        print("  Unknown command. Type h for help.")


def build_arg_parser() -> argparse.ArgumentParser:
    """Create CLI parser for the hardware-default bin calibration tool."""

    parser = argparse.ArgumentParser(description="Interactive rear-bin route fine-tuning for RED_BIN and BLUE_BIN only.")
    parser.add_argument("--file", type=Path, default=BIN_CALIBRATION_FILE, help=f"Calibration JSON path (default: {BIN_CALIBRATION_FILE})")
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--hardware",
        dest="hardware",
        action="store_true",
        default=True,
        help="Connect to serial hardware (default). Movement still requires explicit command confirmations.",
    )
    mode_group.add_argument(
        "--dry-run",
        "--no-hardware",
        dest="hardware",
        action="store_false",
        help="Offline editing/validation mode with no serial connection or motor movement.",
    )
    parser.add_argument("--port", default=SERIAL_PORT, help=f"Serial port for hardware mode (default: {SERIAL_PORT})")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help=f"Serial baud for hardware mode (default: {SERIAL_BAUD})")
    parser.add_argument("--validate-only", action="store_true", help="Validate all loaded/initialized waypoints and exit without saving or opening serial.")
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments; hardware is enabled unless dry-run/no-hardware is requested."""

    return build_arg_parser().parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    global BIN_CALIBRATION_FILE

    args = parse_args(argv)

    BIN_CALIBRATION_FILE = args.file

    print("╔══════════════════════════════════════════════════════════════╗")
    print("║      REAR BIN ROUTE FINE-TUNING — RED_BIN / BLUE_BIN        ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  Hardware mode is default: serial opens unless --dry-run used.║")
    print("║  Use limp/capture/lock to hand-guide waypoints like touch cal.║")
    print("║  Saves require explicit SAVE and create timestamped backups.  ║")
    print("╚══════════════════════════════════════════════════════════════╝")

    data, messages = load_or_initialize_route_schema(BIN_CALIBRATION_FILE)
    for message in messages:
        print(f"[LOAD] {message}")
    print("[LOAD] Editable route is constrained to RED_BIN and BLUE_BIN only.")

    arm = ArmIK(rear_base_yaw_limit_deg=data.get("rear_base_yaw_limit_deg", DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG))
    results = validate_all_waypoints(data, arm)
    for spec in EDITABLE_WAYPOINTS:
        print_validation_result(spec, results[spec.key])

    if args.validate_only:
        return 0 if all(result["ok"] for result in results.values()) else 1

    ser = None
    try:
        if args.hardware:
            print("[MODE] Hardware mode (default). Use --dry-run or --no-hardware for offline editing.")
            ser = _open_serial(args.port, args.baud)
        else:
            print("[MODE] Dry-run/no-hardware mode. No serial connection will be opened and motors cannot move.")
        interactive_loop(data, hardware=args.hardware, ser=ser)
    except KeyboardInterrupt:
        print("\nInterrupted. Unsaved in-memory edits were not written unless SAVE completed earlier.")
        return 130
    finally:
        if ser is not None:
            try:
                ser.close()
                print("Serial connection closed.")
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
