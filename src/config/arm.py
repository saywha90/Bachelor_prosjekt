"""
arm.py
=========
Physical coordinates and sorting logic for the robotic arm workspace.

All coordinates are in centimetres, relative to the shoulder joint origin.
    x = forward (away from the arm base)
    y = left / right
    z = up / down (0 = table surface)

Author: Bachelor Project 2026 – Autonomia
"""

from __future__ import annotations

import json
import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Optional

logger = logging.getLogger(__name__)


class RouteCalibrationError(ValueError):
    """Raised when route-oriented bin calibration is invalid or incomplete."""


@dataclass(frozen=True)
class RoutePose:
    """Cartesian route pose plus optional wrist trim and sag mode."""

    x: float
    y: float
    z: float
    m4_offset: int = 0
    skip_sag: bool = False

    def as_strict_pose(self, m5: int | None = None) -> dict:
        """Return a dict accepted by ArmIK.solve_strict()."""
        pose = {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "m4_offset": self.m4_offset,
            "skip_sag": self.skip_sag,
        }
        if m5 is not None:
            pose["m5"] = m5
        return pose


@dataclass(frozen=True)
class BinRoute:
    """Route-oriented production drop data for a single destination bin."""

    drop: RoutePose


@dataclass(frozen=True)
class TransportRouteCalibration:
    """Validated route-oriented bin calibration."""

    schema_version: int
    shared_waypoints: dict[str, RoutePose]
    bins: dict[str, BinRoute]
    source_schema: str
    rear_base_yaw_limit_deg: float

# ── Height calibration cache ──────────────────────────────────────────
_height_calibration_cache: Optional[List[dict]] = None
_height_calibration_loaded: bool = False

CALIBRATION_FILE = Path(__file__).resolve().parent.parent / "calibration" / "homography_calibration.json"
BIN_CALIBRATION_FILE = Path(__file__).resolve().parent.parent / "calibration" / "bin_calibration.json"


def load_height_calibration() -> Optional[List[dict]]:
    """Load height_calibration data from homography_calibration.json.

    Returns a list of ``{"distance": float, "z": float}`` entries sorted
    by distance, or ``None`` if the file doesn't exist or doesn't contain
    height calibration data.  The result is cached so the file is only
    read once per process.
    """
    global _height_calibration_cache, _height_calibration_loaded
    if _height_calibration_loaded:
        return _height_calibration_cache

    _height_calibration_loaded = True
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
        raw = data.get("height_calibration")
        if raw and isinstance(raw, list) and len(raw) >= 2:
            # Sort by distance for interpolation
            _height_calibration_cache = sorted(raw, key=lambda p: p["distance"])
            logger.info(
                "[CONFIG] Loaded height calibration with %d points from %s",
                len(_height_calibration_cache), CALIBRATION_FILE,
            )
        else:
            logger.info("[CONFIG] No height_calibration in %s — using formula fallback.", CALIBRATION_FILE)
    except (FileNotFoundError, json.JSONDecodeError, KeyError) as exc:
        logger.info("[CONFIG] Could not load height calibration (%s) — using formula fallback.", exc)

    return _height_calibration_cache

# ── Bin positions ─────────────────────────────────────────────────────
#   Bins are placed in front of the arm at different X distances with Y≈0.
#   This keeps M1 stable (no base rotation) during the sort motion.
#   Z is set high enough so the arm clears the bin walls before dropping.
BINS = {
    "RED_BIN":    (20.0,  -3.0,  15.0),   # reachable forward-left
    "BLUE_BIN":   (20.0,   3.0,  15.0),   # reachable forward-right
    "REJECT_BIN": (22.0,   0.0,  12.0),   # reachable straight ahead
}

# ── Home / resting position ──────────────────────────────────────────
#   A safe, tucked-in position where the arm waits between cycles.
#   NOTE: Previously (10, 0, 15) caused extreme elbow folding (m3 = 273)
#   which overloaded motor ID 3 (XM430) and caused red blinking.
#   (20, 0, 30) keeps the shoulder high so the claw stays well above the
#   desk surface.  Reduced X + raised Z = shoulder tilts more upright.
HOME_POSITION = (20.0, 0.0, 30.0)

# ── Grab / drop heights ──────────────────────────────────────────────
GRAB_HEIGHT      = 2.0   # z when closing the claw (target center of ball on table)
CLEARANCE_HEIGHT = 15.0  # Reduced from 28.0; 15 is plenty of room and prevents IK clamping at far reaches

# ── Distance-based grab height adjustment ─────────────────────────────
#   The arm sags/flexes at every reach distance — even close balls can
#   clip the desk if the grab height doesn't account for the horizontal
#   distance.  This linear model raises the grab height proportionally
#   to distance so the claw maintains safe clearance everywhere.
#
#   grab_z = GRAB_HEIGHT + distance * GRAB_HEIGHT_SLOPE
#
#   Tuning:
#     GRAB_HEIGHT_SLOPE — cm of extra Z per cm of horizontal distance.
#                         Start with 0.05 and increase if the claw still
#                         scrapes at any distance.
#     GRAB_HEIGHT_MAX   — cap so the arm doesn't lift too high and miss
#                         the ball entirely.
GRAB_HEIGHT_SLOPE     = 0.05   # cm extra Z per cm of horizontal distance (applied everywhere)
GRAB_HEIGHT_MAX       = 5.0    # cm – absolute maximum grab height

# ──────────────────────────────────────────────────────────────────────
# DEPRECATED — APPROACH_HEIGHT
#   Was used by the old 2-step approach (descend to approach height,
#   then to grab height).  Replaced with a single direct move to
#   GRAB_HEIGHT (see ADR-003).  Use compute_grab_height() instead.
#   Retained *only* for calibration script 08_pick_test.py which still
#   uses it as an intermediate safety height during manual testing.
# ──────────────────────────────────────────────────────────────────────
APPROACH_HEIGHT  = 24.0

# ── Timing (seconds) ─────────────────────────────────────────────────
GRAB_DWELL    = 0.8   # time to wait while the claw closes
RELEASE_DWELL = 0.5   # time to wait while the claw opens
SCAN_INTERVAL = 2.0   # pause between reaching SCAN_POSE and capturing a frame

# ── Camera-to-shoulder coordinate correction ─────────────────────────
# The homography WORKSPACE_CM now maps pixels directly to the shoulder
# joint (motor 2 pivot) coordinate frame.  No additional offset is
# needed — the 4 calibration corners in vision_bridge.py are measured
# in cm from the shoulder joint itself.
#
# Previously CAMERA_OFFSET_X was 25.5 cm because the homography corners
# were measured from the external camera stand (fixed-camera setup), causing double-counting.
#
# If you still see a small systematic error after re-calibrating the
# homography, you can add a fine-tuning offset here (typically < 3 cm).
# Use  python src/calibration/09_touch_calibration.py  to recalibrate.
CAMERA_OFFSET_X = 1.0     # +6 cm forward offset — arm was reaching 6 cm short (2026-05-18)
CAMERA_OFFSET_Y = 0.0     # Reset to zero — re-calibrate via 09_touch_calibration.py (2026-04-28)
CAMERA_HEIGHT   = 50.0   # cm – camera lens height above table surface

# ── Wrist-mounted camera scan pose ──────────────────────────────────
# Joint positions (Dynamixel steps) where the arm parks the
# wrist-mounted camera so it sees the entire workspace from above.
#
# Tuning notes (wrist-mounted OAK-D S2):
#   m1 (base)     — keep centred (2048) so camera looks straight forward
#   m2 (shoulder) — lifted up so the wrist sits ~30–40 cm above desk
#   m3 (elbow)    — folded back so the forearm tilts the camera downward
#   m4 (wrist)    — angled so the camera optical axis points at the desk
#                   (NOT along the claw direction — see calibration step 02c)
#   m5 (claw)     — open and out of camera view
#
# MUST BE TUNED EMPIRICALLY for your specific camera mount geometry.
# See docs/calibration.md → Step 02c.
SCAN_POSE = {
    "m1": 2048,
    "m2": 2750,   # shoulder raised — wrist ~35 cm above desk (synced with homography_calibration.json)
    "m3": 600,    # elbow folded — forearm angled down toward desk
    "m4": 1000,    # wrist tilted — camera optical axis points at workspace (synced with homography_calibration.json)
    "m5": 2745,   # claw open and out of camera view
}

# Tolerance for verifying the arm is actually at SCAN_POSE before
# running vision (Dynamixel steps; ~1.8° at 4096 steps/360°)
SCAN_POSE_TOLERANCE = 50

# ── Motion profile for startup home move ─────────────────────────────
# Dynamixel profile velocity (~0.229 rpm per unit) and acceleration
# (~214 rev/min² per unit) used when moving to SCAN_POSE on startup.
# These produce a smooth trapezoidal velocity profile: the arm accelerates
# gently, holds a moderate speed, then decelerates before stopping.
# Increase VEL to move faster; decrease ACC for a softer start/stop.
STARTUP_PROFILE_VEL = 60    # fast but controlled (not sluggish, not instant)
STARTUP_PROFILE_ACC = 15    # gentle ramp-up / ramp-down

# ── IK pitch limit ───────────────────────────────────────────────────
# Maximum wrist pitch angle (radians) the IK solver will try when
# tilting the claw forward to extend reach.  The pitch search starts
# at −π/2 (straight down) and increments toward this value.
# 0.0 = horizontal; negative values limit the tilt before horizontal.
MAX_REACH_PITCH = 0.0

# ── Rear-placement route base yaw guard ───────────────────────────────
# Rear bins are reached by folding the shoulder/forearm over the base, not
# by spinning M1 around to face the rear.  Route-schema calibration may
# override this symmetric limit with rear_base_yaw_limit_deg.
DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG = 45.0

# ── Claw motor positions (Dynamixel steps) ─────────────────────────
CLAW_OPEN_POS   = 2745    # open/neutral position for gripper (M5 XM430-W210 raw goal position)
CLAW_CLOSED_POS = 3350    # safe closed/grip limit (extended to 3350 for tighter grip on small balls)

# ── Grip verification ──────────────────────────────────────────────
GRIP_VERIFY_TOLERANCE = 30        # Dynamixel steps: if claw pos is within this of CLAW_CLOSED_POS, grip failed
GRIP_LOAD_THRESHOLD   = 15        # Minimum absolute load value to confirm grip (lowered: 50 mA limit makes motor more sensitive)
MAX_PICK_RETRIES      = 2         # Number of re-grab attempts before skipping a ball
VERIFY_HEIGHT         = 8.0       # cm – height to lift to for grip verification (between GRAB_HEIGHT and CLEARANCE_HEIGHT)

# ── Adaptive grip settings ──────────────────────────────────────────
GRIP_CURRENT_LIMIT    = 50        # mA – max current for M5 during grip (low for sensitivity; protects 3D-printed claw)
GRIP_PROFILE_VEL      = 80        # fast closing velocity (matches normal operating speed)
GRIP_PROFILE_ACC      = 20        # moderate closing acceleration (normal is 20; was 10)
GRIP_POLL_INTERVAL    = 0.05      # seconds between load readings during adaptive grip
GRIP_TIMEOUT          = 15.0      # max seconds to wait for grip to complete (555 steps ÷ 30/step ≈ 18 increments × ~0.4s ≈ 7.5s minimum)
GRIP_LOAD_DETECT      = 5         # sensitive load threshold: light resistance means ball contact (lowered for 50 mA limit)
GRIP_POSITION_STALL   = 8         # sensitive stall threshold: small movement under command means contact
GRIP_EXTRA_CLOSE      = 50        # step increment size during adaptive close (was 30; larger = faster close)
EXPECTED_BALL_DIAMETER_CM = 5.0   # production balls are 50 mm / 5 cm diameter
GRIP_MIN_BALL_BLOCKED_STEPS = 30  # position-only detection: min gap from empty-closed (was 60, too strict for smaller balls)
GRIP_MIN_BLOCKED_WITH_SENSOR = 5  # sensor-assisted threshold: when load/current confirm, accept very small gaps (not at end-stop)
DEFAULT_PROFILE_VEL   = 80        # normal operating velocity
DEFAULT_PROFILE_ACC   = 20        # normal operating acceleration
M5_DEFAULT_CURRENT_LIMIT = 1193   # M5 XM430-W210 factory default current limit in mA


def _interpolate_field(distance: float, calibration: List[dict], field: str) -> float:
    """Generic clamped linear interpolation over a sorted calibration list.

    Parameters
    ----------
    distance : float
        The horizontal distance to interpolate at.
    calibration : list of dict
        Sorted list of ``{"distance": float, field: float}`` entries.
    field : str
        The dictionary key to interpolate (e.g. ``"z"`` or ``"m4_offset"``).

    Returns
    -------
    float
        Interpolated value.  If *distance* is outside the calibrated range
        the nearest endpoint value is returned (clamped extrapolation).
    """
    if distance <= calibration[0]["distance"]:
        return float(calibration[0][field])
    if distance >= calibration[-1]["distance"]:
        return float(calibration[-1][field])

    for j in range(len(calibration) - 1):
        d0 = calibration[j]["distance"]
        v0 = calibration[j][field]
        d1 = calibration[j + 1]["distance"]
        v1 = calibration[j + 1][field]
        if d0 <= distance <= d1:
            t = (distance - d0) / (d1 - d0) if d1 != d0 else 0.0
            return v0 + t * (v1 - v0)

    # Fallback (should not happen)
    return float(calibration[-1][field])


def _interpolate_height(distance: float, calibration: List[dict]) -> float:
    """Linearly interpolate the grab Z from calibrated (distance, z) pairs."""
    return _interpolate_field(distance, calibration, "z")


def compute_grab_height(x: float, y: float) -> float:
    """Return the optimal grab Z (cm) for a ball at position (x, y).

    **Calibrated mode** (preferred): If ``height_calibration`` data exists
    in ``homography_calibration.json``, uses linear interpolation between
    the calibrated (distance, z) points recorded during touch calibration.

    **Formula fallback**: ``grab_z = GRAB_HEIGHT + distance * GRAB_HEIGHT_SLOPE``,
    capped at ``GRAB_HEIGHT_MAX``.

    Parameters
    ----------
    x, y : float
        Ball position in arm-frame centimetres (x = forward, y = left/right).

    Returns
    -------
    float
        The Z coordinate (cm above desk) the arm should descend to for
        a clean grab at this distance.
    """
    distance = math.sqrt(x ** 2 + y ** 2)

    calibration = load_height_calibration()
    if calibration is not None:
        grab_z = _interpolate_height(distance, calibration)
        logger.info(
            "[CONFIG] Grab height for distance %.1f cm: %.2f cm "
            "(interpolated from %d calibration points)",
            distance, grab_z, len(calibration),
        )
        return grab_z

    # Formula fallback
    extra = distance * GRAB_HEIGHT_SLOPE
    grab_z = GRAB_HEIGHT + extra
    grab_z = min(grab_z, GRAB_HEIGHT_MAX)
    logger.info(
        "[CONFIG] Grab height for distance %.1f cm: %.2f cm "
        "(formula: base=%.1f + extra=%.2f, capped at %.1f)",
        distance, grab_z, GRAB_HEIGHT, extra, GRAB_HEIGHT_MAX,
    )
    return grab_z


# ── Bin calibration cache ─────────────────────────────────────────────
_bin_cal_cache: dict | None = None
_bin_cal_loaded: bool = False
_route_cal_cache: TransportRouteCalibration | None = None
_route_cal_loaded: bool = False

REQUIRED_SHARED_ROUTE_WAYPOINTS = ("front_neutral", "rear_transfer")
REAR_RETURN_LIFT_WAYPOINT = "rear_return_lift"
REQUIRED_BIN_ROUTE_POSES = ("drop",)
REAR_RETURN_LIFT_Z_INCREMENT_CM = 4.0


def load_bin_calibration() -> dict | None:
    """Load bin calibration from bin_calibration.json.
    Returns the 'bins' dict or None if file not found.
    Caches result after first load.
    """
    global _bin_cal_cache, _bin_cal_loaded
    if _bin_cal_loaded:
        return _bin_cal_cache
    _bin_cal_loaded = True
    if BIN_CALIBRATION_FILE.exists():
        try:
            with open(BIN_CALIBRATION_FILE, "r") as f:
                data = json.load(f)
            _bin_cal_cache = data.get("bins", None)
        except (json.JSONDecodeError, OSError) as exc:
            print(f"[arm.py] WARNING: failed to load bin calibration: {exc}")
            _bin_cal_cache = None
    return _bin_cal_cache


def _normalise_bin_key(color_string: str) -> str:
    key = color_string.upper().strip()
    if not key.endswith("_BIN"):
        key += "_BIN"
    return key


def _read_bin_calibration_file(path: Path | None = None) -> dict:
    cal_path = path or BIN_CALIBRATION_FILE
    try:
        with open(cal_path, "r") as f:
            return json.load(f)
    except FileNotFoundError as exc:
        raise RouteCalibrationError(f"Bin route calibration file not found: {cal_path}") from exc
    except json.JSONDecodeError as exc:
        raise RouteCalibrationError(f"Invalid JSON in bin route calibration file {cal_path}: {exc}") from exc
    except OSError as exc:
        raise RouteCalibrationError(f"Could not read bin route calibration file {cal_path}: {exc}") from exc


def _pose_from_mapping(raw: Any, context: str) -> RoutePose:
    if not isinstance(raw, dict):
        raise RouteCalibrationError(f"{context} must be an object with x, y, z fields")
    missing = [key for key in ("x", "y", "z") if key not in raw]
    if missing:
        raise RouteCalibrationError(f"{context} missing required field(s): {', '.join(missing)}")
    try:
        return RoutePose(
            x=float(raw["x"]),
            y=float(raw["y"]),
            z=float(raw["z"]),
            m4_offset=int(raw.get("m4_offset", 0) or 0),
            skip_sag=bool(raw.get("skip_sag", False)),
        )
    except (TypeError, ValueError) as exc:
        raise RouteCalibrationError(f"{context} has invalid numeric field: {exc}") from exc


def _parse_rear_base_yaw_limit(raw: Any, context: str = "rear_base_yaw_limit_deg") -> float:
    try:
        limit = abs(float(raw))
    except (TypeError, ValueError) as exc:
        raise RouteCalibrationError(f"{context} must be numeric") from exc
    if not math.isfinite(limit) or limit < 0.0 or limit > 180.0:
        raise RouteCalibrationError(f"{context} must be finite and within [0, 180] degrees")
    return limit


def _rear_fold_base_yaw_deg(pose: RoutePose) -> float:
    if abs(pose.x) < 1e-9 and abs(pose.y) < 1e-9:
        return 0.0
    yaw = math.degrees(math.atan2(pose.y, pose.x) + math.pi)
    return (yaw + 180.0) % 360.0 - 180.0


def _validate_rear_route_yaw(name: str, pose: RoutePose, limit_deg: float) -> None:
    yaw_deg = _rear_fold_base_yaw_deg(pose)
    if yaw_deg < -limit_deg - 1e-9 or yaw_deg > limit_deg + 1e-9:
        raise RouteCalibrationError(
            f"Rear route waypoint {name} requires base yaw {yaw_deg:.1f}° outside "
            f"configured range [-{limit_deg:.1f}°, {limit_deg:.1f}°]"
        )


def _parse_route_calibration(data: dict, source_schema: str) -> TransportRouteCalibration:
    version = data.get("schema_version", data.get("version"))
    try:
        schema_version = int(version)
    except (TypeError, ValueError) as exc:
        raise RouteCalibrationError("Route calibration missing integer schema_version") from exc
    if schema_version < 2:
        raise RouteCalibrationError(f"Unsupported route calibration schema_version={schema_version}")

    rear_base_yaw_limit_deg = _parse_rear_base_yaw_limit(
        data.get("rear_base_yaw_limit_deg", DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG)
    )

    shared_raw = data.get("shared_waypoints")
    if not isinstance(shared_raw, dict):
        raise RouteCalibrationError("Route calibration missing shared_waypoints object")
    missing_shared = [name for name in REQUIRED_SHARED_ROUTE_WAYPOINTS if name not in shared_raw]
    if missing_shared:
        raise RouteCalibrationError(
            f"Route calibration missing required shared waypoint(s): {', '.join(missing_shared)}"
        )
    shared = {
        name: _pose_from_mapping(raw, f"shared_waypoints.{name}")
        for name, raw in shared_raw.items()
    }
    if REAR_RETURN_LIFT_WAYPOINT not in shared:
        rear_transfer = shared["rear_transfer"]
        shared[REAR_RETURN_LIFT_WAYPOINT] = RoutePose(
            x=rear_transfer.x,
            y=rear_transfer.y,
            z=rear_transfer.z + REAR_RETURN_LIFT_Z_INCREMENT_CM,
            m4_offset=rear_transfer.m4_offset,
            skip_sag=rear_transfer.skip_sag,
        )

    bins_raw = data.get("bins")
    if not isinstance(bins_raw, dict) or not bins_raw:
        raise RouteCalibrationError("Route calibration missing bins object")

    bins: dict[str, BinRoute] = {}
    for raw_key, raw_entry in bins_raw.items():
        key = _normalise_bin_key(raw_key)
        if not isinstance(raw_entry, dict):
            raise RouteCalibrationError(f"bins.{key} must be an object")
        missing_poses = [name for name in REQUIRED_BIN_ROUTE_POSES if name not in raw_entry]
        if missing_poses:
            raise RouteCalibrationError(
                f"bins.{key} missing required route pose(s): {', '.join(missing_poses)}"
            )
        # v2 schemas may still have "approach" — silently ignore it
        bins[key] = BinRoute(
            drop=_pose_from_mapping(raw_entry["drop"], f"bins.{key}.drop"),
        )

    _validate_rear_route_yaw("shared_waypoints.rear_transfer", shared["rear_transfer"], rear_base_yaw_limit_deg)
    _validate_rear_route_yaw(
        f"shared_waypoints.{REAR_RETURN_LIFT_WAYPOINT}",
        shared[REAR_RETURN_LIFT_WAYPOINT],
        rear_base_yaw_limit_deg,
    )
    for key, route in bins.items():
        _validate_rear_route_yaw(f"bins.{key}.drop", route.drop, rear_base_yaw_limit_deg)

    return TransportRouteCalibration(
        schema_version=schema_version,
        shared_waypoints=shared,
        bins=bins,
        source_schema=source_schema,
        rear_base_yaw_limit_deg=rear_base_yaw_limit_deg,
    )


def _legacy_route_from_bins(bins: dict) -> TransportRouteCalibration:
    if not isinstance(bins, dict) or not bins:
        raise RouteCalibrationError("Legacy bin calibration missing bins object")

    # Legacy schema has one calibrated target per bin and no rear-transfer
    # semantics.  Provide explicit route objects for loader compatibility only;
    # production requires the route schema via require_route_schema=True.
    shared = {
        "front_neutral": RoutePose(20.0, 0.0, CLEARANCE_HEIGHT),
        "rear_transfer": RoutePose(20.0, 0.0, CLEARANCE_HEIGHT),
    }
    routes: dict[str, BinRoute] = {}
    for raw_key, entry in bins.items():
        key = _normalise_bin_key(raw_key)
        pose = _pose_from_mapping(entry, f"bins.{key}")
        routes[key] = BinRoute(drop=pose)
    return TransportRouteCalibration(
        schema_version=1,
        shared_waypoints=shared,
        bins=routes,
        source_schema="legacy",
        rear_base_yaw_limit_deg=DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG,
    )


def load_transport_route_calibration(
    path: Path | None = None,
    *,
    require_route_schema: bool = False,
) -> TransportRouteCalibration:
    """Load validated route-oriented transport calibration.

    Supports both schemas in ``bin_calibration.json``.  If a versioned route
    schema is present, it is preferred and strictly validated.  Legacy schema
    is still loadable for compatibility unless ``require_route_schema`` is set,
    which production sorting uses to fail closed instead of guessing routes.
    """
    global _route_cal_cache, _route_cal_loaded
    if path is None and _route_cal_loaded and _route_cal_cache is not None:
        if require_route_schema and _route_cal_cache.source_schema != "route":
            raise RouteCalibrationError(
                "Production transport requires route schema calibration; legacy single-target bin schema is insufficient"
            )
        return _route_cal_cache

    data = _read_bin_calibration_file(path)
    if not isinstance(data, dict):
        raise RouteCalibrationError("Bin calibration root must be an object")

    has_route_schema = "shared_waypoints" in data or "schema_version" in data or "version" in data
    if has_route_schema:
        route = _parse_route_calibration(data, "route")
    else:
        route = _legacy_route_from_bins(data.get("bins"))

    if require_route_schema and route.source_schema != "route":
        raise RouteCalibrationError(
            "Production transport requires route schema calibration; legacy single-target bin schema is insufficient"
        )

    if path is None:
        _route_cal_cache = route
        _route_cal_loaded = True
    return route


def get_transport_route(color_string: str) -> list[tuple[str, RoutePose]]:
    """Return strict production route waypoints for a destination colour."""
    route = load_transport_route_calibration(require_route_schema=True)
    key = _normalise_bin_key(color_string)
    if key not in route.bins:
        raise RouteCalibrationError(f"No route calibration for destination bin {key}")
    bin_route = route.bins[key]
    return [
        ("front_neutral", route.shared_waypoints["front_neutral"]),
        ("rear_transfer", route.shared_waypoints["rear_transfer"]),
        (f"{key}.drop", bin_route.drop),
    ]


def get_transport_return_route() -> list[tuple[str, RoutePose]]:
    """Return the open-claw retreat route from a rear bin back toward front-facing posture."""
    route = load_transport_route_calibration(require_route_schema=True)
    return [
        (REAR_RETURN_LIFT_WAYPOINT, route.shared_waypoints[REAR_RETURN_LIFT_WAYPOINT]),
        ("front_neutral", route.shared_waypoints["front_neutral"]),
    ]


# ── Wrist calibration cache ───────────────────────────────────────────
_wrist_calibration_cache: Optional[List[dict]] = None
_wrist_calibration_loaded: bool = False


def load_wrist_calibration() -> Optional[List[dict]]:
    """Load wrist_calibration data from homography_calibration.json.

    Returns a list of ``{"distance": float, "m4_offset": int}`` entries
    sorted by distance, or ``None`` if the file doesn't exist or doesn't
    contain wrist calibration data.  The result is cached so the file is
    only read once per process.
    """
    global _wrist_calibration_cache, _wrist_calibration_loaded
    if _wrist_calibration_loaded:
        return _wrist_calibration_cache

    _wrist_calibration_loaded = True
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
        raw = data.get("wrist_calibration")
        if raw and isinstance(raw, list) and len(raw) >= 2:
            _wrist_calibration_cache = sorted(raw, key=lambda p: p["distance"])
            logger.info(
                "[CONFIG] Loaded wrist calibration with %d points from %s",
                len(_wrist_calibration_cache), CALIBRATION_FILE,
            )
        else:
            logger.info("[CONFIG] No wrist_calibration in %s — wrist correction disabled.", CALIBRATION_FILE)
    except (FileNotFoundError, json.JSONDecodeError, KeyError) as exc:
        logger.info("[CONFIG] Could not load wrist calibration (%s) — wrist correction disabled.", exc)

    return _wrist_calibration_cache


def _interpolate_wrist(distance: float, calibration: List[dict]) -> int:
    """Linearly interpolate the m4 offset from calibrated (distance, m4_offset) pairs."""
    return int(round(_interpolate_field(distance, calibration, "m4_offset")))


def compute_wrist_correction(x: float, y: float) -> int:
    """Return the m4 step offset for a ball at (x, y) using calibrated data.

    Uses linear interpolation between calibrated (distance, m4_offset)
    points recorded during touch calibration.

    Parameters
    ----------
    x, y : float
        Ball position in arm-frame centimetres.

    Returns
    -------
    int
        Dynamixel step offset to add to the IK-computed m4.
        Returns 0 if no calibration data is available.
    """
    distance = math.sqrt(x ** 2 + y ** 2)

    calibration = load_wrist_calibration()
    if calibration is None:
        return 0

    offset = _interpolate_wrist(distance, calibration)
    logger.info(
        "[CONFIG] Wrist correction for distance %.1f cm: %+d steps "
        "(interpolated from %d calibration points)",
        distance, offset, len(calibration),
    )
    return offset


def get_bin_coords(color_string: str) -> tuple:
    """Return (x, y, z) for the given bin colour.

    Checks calibrated positions first, falls back to hardcoded BINS.
    """
    key = _normalise_bin_key(color_string)

    # Try calibrated positions first
    cal = load_bin_calibration()
    if cal and key in cal:
        entry = cal[key]
        return (entry["x"], entry["y"], entry["z"])

    # Fallback to hardcoded
    if key in BINS:
        return BINS[key]
    return BINS["REJECT_BIN"]


def get_bin_m4_offset(color_string: str) -> int:
    """Return the calibrated m4_offset for a bin, or 0 if not calibrated."""
    key = _normalise_bin_key(color_string)
    cal = load_bin_calibration()
    if cal and key in cal:
        return cal[key].get("m4_offset", 0)
    return 0
