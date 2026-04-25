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

import logging

logger = logging.getLogger(__name__)

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
APPROACH_HEIGHT  = 24.0  # z during the 80% XY approach (raised to stay above grab height)
CLEARANCE_HEIGHT = 28.0  # z to lift to before traversing to a bin

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
# Use  python src/calibration/06_homography.py  to recalibrate.
CAMERA_OFFSET_X = 0.0     # homography is shoulder-relative; no offset needed
CAMERA_OFFSET_Y = 0.0     # homography is shoulder-relative; no offset needed
CAMERA_HEIGHT   = 29.0   # cm – camera lens height above table surface (measured 2026-04-25)

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
    "m2": 2750,   # shoulder raised — wrist ~35 cm above desk
    "m3": 750,    # elbow folded — forearm angled down toward desk
    "m4": 1050,   # wrist tilted — camera optical axis points at workspace
    "m5": 2248,   # claw open and out of camera view
}

# Tolerance for verifying the arm is actually at SCAN_POSE before
# running vision (Dynamixel steps; ~1.8° at 4096 steps/360°)
SCAN_POSE_TOLERANCE = 20

# ── Motion profile for startup home move ─────────────────────────────
# Dynamixel profile velocity (~0.229 rpm per unit) and acceleration
# (~214 rev/min² per unit) used when moving to SCAN_POSE on startup.
# These produce a smooth trapezoidal velocity profile: the arm accelerates
# gently, holds a moderate speed, then decelerates before stopping.
# Increase VEL to move faster; decrease ACC for a softer start/stop.
STARTUP_PROFILE_VEL = 60    # fast but controlled (not sluggish, not instant)
STARTUP_PROFILE_ACC = 15    # gentle ramp-up / ramp-down


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
        ``(x, y, z)`` coordinates of the bin.

    Raises
    ------
    KeyError
        If no matching bin is found, falls back to ``REJECT_BIN``.
    """
    key = color_string.upper().strip()
    if not key.endswith("_BIN"):
        key += "_BIN"

    if key in BINS:
        return BINS[key]

    logger.warning("[CONFIG] ⚠️  Unknown colour '%s' → routing to REJECT_BIN", color_string)
    return BINS["REJECT_BIN"]
