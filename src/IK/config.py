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
    "RED_BIN":    (20.0,  15.0,  10.0),
    "BLUE_BIN":   (20.0, -15.0,  10.0),
    "REJECT_BIN": (10.0,  15.0,  10.0),
}

# ── Home / resting position ──────────────────────────────────────────
#   A safe, tucked-in position where the arm waits between cycles.
HOME_POSITION = (10.0, 0.0, 15.0)

# ── Grab / drop heights ──────────────────────────────────────────────
GRAB_HEIGHT    = 0.0     # z when closing the claw on an object
CLEARANCE_HEIGHT = 12.0  # z to lift to before traversing to a bin

# ── Timing (seconds) ─────────────────────────────────────────────────
GRAB_DWELL   = 0.8   # time to wait while the claw closes
RELEASE_DWELL = 0.5   # time to wait while the claw opens


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
