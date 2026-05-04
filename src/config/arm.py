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

import json
import logging
import math
from pathlib import Path
from typing import List, Optional

logger = logging.getLogger(__name__)

# ── Height calibration cache ──────────────────────────────────────────
_height_calibration_cache: Optional[List[dict]] = None
_height_calibration_loaded: bool = False

CALIBRATION_FILE = Path(__file__).resolve().parent.parent / "calibration" / "homography_calibration.json"


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
#   Z is set high enough so the arm clears the bin walls before dropping.
BINS = {
    "RED_BIN":    (20.0,   8.0,  10.0),   # Y reduced from 15 to 8 to reduce left swing
    "BLUE_BIN":   (20.0,  -8.0,  10.0),   # Y reduced from -15 to -8 symmetrically
    "REJECT_BIN": (25.0,   0.0,  12.0),
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
CAMERA_OFFSET_X = 0.0     # Reset to zero — re-calibrate via 09_touch_calibration.py (2026-04-28)
CAMERA_OFFSET_Y = 0.0     # Reset to zero — re-calibrate via 09_touch_calibration.py (2026-04-28)
CAMERA_HEIGHT   = 56.5   # cm – camera lens height above table surface

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
    "m2": 2800,   # shoulder raised — wrist ~35 cm above desk
    "m3": 950,    # elbow folded — forearm angled down toward desk
    "m4": 800,   # wrist tilted — camera optical axis points at workspace
    "m5": 2048,   # claw open and out of camera view
}

# Tolerance for verifying the arm is actually at SCAN_POSE before
# running vision (Dynamixel steps; ~1.8° at 4096 steps/360°)
SCAN_POSE_TOLERANCE = 20

# ── M3 thermal protection in SCAN_POSE ───────────────────────────────
# Motor 3 (XM430-W350, elbow) bears a heavy static gravity load in
# SCAN_POSE (forearm + camera folded back ~92°).  At 0.47 A continuous
# draw, it reaches concerning temperatures after ~5 minutes.
#
# Mitigation 1 — Reduced hold current:
#   Lower the Current Limit (Dynamixel register 38) on M3 when the arm
#   is parked at SCAN_POSE.  The motor only needs to *hold* position,
#   not accelerate, so a lower limit reduces heat with minimal position
#   drift.  Value is in mA (XM430: 1 unit ≈ 1 mA; max stall ~1400 mA).
#   0 = disabled (use default / max current).
#   300 mA is only a cautious starting point; treat it as experimental
#   until runtime current/drift checks have been validated on hardware.
M3_SCAN_CURRENT_LIMIT = 400       # mA — scan-hold current cap for M3 to prevent overheating
M3_DEFAULT_CURRENT_LIMIT = 1193  # mA — XM430-W350 factory default current limit

# Mitigation 2 — Periodic torque relaxation (disabled by default):
#   Torque-off at SCAN_POSE is mechanically unsafe unless a validated rest
#   pose and recovery path exist.  Keep this path disabled until such a
#   pose has been proven on hardware.
M3_TORQUE_RELAX_ENABLED = False
M3_RELAX_REST_POSE = None
M3_RELAX_INTERVAL = 45.0   # seconds — only used if torque relaxation is explicitly enabled
M3_RELAX_DURATION = 3.0    # seconds — only used if torque relaxation is explicitly enabled

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

# ── Claw motor positions (Dynamixel steps) ─────────────────────────
CLAW_OPEN_POS   = 2016    # open/neutral position for gripper
CLAW_CLOSED_POS = 2890    # closed/grip position (tune on real hardware — must be the EMPTY jaws-touching position)

# ── Grip verification ──────────────────────────────────────────────
GRIP_VERIFY_TOLERANCE = 30        # Dynamixel steps: if claw pos is within this of CLAW_CLOSED_POS, grip failed
GRIP_LOAD_THRESHOLD   = 50        # Minimum absolute load value to confirm grip (from read_load)
MAX_PICK_RETRIES      = 2         # Number of re-grab attempts before skipping a ball
VERIFY_HEIGHT         = 8.0       # cm – height to lift to for grip verification (between GRAB_HEIGHT and CLEARANCE_HEIGHT)

# ── Adaptive grip settings ──────────────────────────────────────────
GRIP_CURRENT_LIMIT    = 200       # mA – max current for M5 during grip (protects 3D-printed claw)
GRIP_PROFILE_VEL      = 30        # slow closing velocity (normal is 80)
GRIP_PROFILE_ACC      = 10        # slow closing acceleration (normal is 20)
GRIP_POLL_INTERVAL    = 0.05      # seconds between load readings during adaptive grip
GRIP_TIMEOUT          = 3.0       # max seconds to wait for grip to complete
GRIP_LOAD_DETECT      = 40        # load threshold to detect object contact (lower than GRIP_LOAD_THRESHOLD)
GRIP_POSITION_STALL   = 5         # if position changes less than this over 2 readings, consider stalled
GRIP_EXTRA_CLOSE      = 30        # extra steps to close past contact point for secure hold
DEFAULT_PROFILE_VEL   = 80        # normal operating velocity
DEFAULT_PROFILE_ACC   = 20        # normal operating acceleration
M5_DEFAULT_CURRENT_LIMIT = 1193   # XM430-W350 factory default current limit in mA


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
    """Return the (x, y, z) coordinates for the named bin.

    Parameters
    ----------
    color_string : str
        A colour label such as ``"red"``, ``"BLUE"``, or ``"Red"``.
        The lookup is case-insensitive and appends ``_BIN`` automatically
        if needed.

    Returns
    -------
    tuple
        ``(x, y, z)`` coordinates of the bin.  If no matching bin is
        found, silently falls back to ``REJECT_BIN`` and logs a warning.
    """
    key = color_string.upper().strip()
    if not key.endswith("_BIN"):
        key += "_BIN"

    if key in BINS:
        return BINS[key]

    logger.warning("[CONFIG] ⚠️  Unknown colour '%s' → routing to REJECT_BIN", color_string)
    return BINS["REJECT_BIN"]
