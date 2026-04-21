"""
config.py
=========
Physical coordinates and sorting logic for the robotic arm workspace.

All coordinates are in centimetres, relative to the shoulder joint origin.
    x = forward (away from the arm base)
    y = left / right
    z = up / down (0 = table surface)
"""

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
GRAB_HEIGHT      = 13.0  # z when closing the claw (floor is 6.0, raised to 13.0)Scan privilegeScan
APPROACH_HEIGHT  = 24.0  # z during the 80% XY approach (raised to stay above grab height)
CLEARANCE_HEIGHT = 28.0  # z to lift to before traversing to a bin

# ── Timing (seconds) ─────────────────────────────────────────────────
GRAB_DWELL   = 0.8   # time to wait while the claw closes
RELEASE_DWELL = 0.5   # time to wait while the claw opens

# ── Camera-to-shoulder coordinate correction ─────────────────────────
# The homography WORKSPACE_CM now maps pixels directly to the shoulder
# joint (motor 2 pivot) coordinate frame.  No additional offset is
# needed — the 4 calibration corners in vision_bridge.py are measured
# in cm from the shoulder joint itself.
#
# Previously CAMERA_OFFSET_X was 25.5 cm because the homography corners
# were measured from the camera pillar/base, causing double-counting.
#
# If you still see a small systematic error after re-calibrating the
# homography, you can add a fine-tuning offset here (typically < 3 cm).
# Use  python3 calibrate_homography.py  to recalibrate.
CAMERA_OFFSET_X = 0.0     # homography is shoulder-relative; no offset needed
CAMERA_OFFSET_Y = 0.0     # homography is shoulder-relative; no offset needed
CAMERA_HEIGHT   = 43.0   # cm – camera lens height above table surface


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

    print(f"[CONFIG] ⚠️  Unknown colour '{color_string}' → routing to REJECT_BIN")
    return BINS["REJECT_BIN"]
