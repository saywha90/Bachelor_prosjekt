# ADR-004: Touch Calibration Replaces Ruler-Based Homography

## Status

Accepted

## Date

2026-04-25

## Context

The pixel-to-cm homography calibration (Step 6) previously required the user to manually measure the physical (x, y) coordinates of 4 workspace corners using a ruler from the shoulder joint (motor 2 pivot). This approach (`06_homography.py`) had several accuracy problems:

1. **Ruler measurement error** — measuring from the shoulder pivot to a corner 25–30 cm away introduces ±1–2 cm error, especially for the y-axis (left/right) where parallax makes ruler alignment difficult.
2. **Coordinate frame misalignment** — the user must mentally map "distance from shoulder joint" to the IK solver's coordinate frame. Any misunderstanding (e.g., measuring from the arm base rather than the shoulder pivot) produces a systematic offset.
3. **Non-repeatability** — different operators measure differently, producing different homographies for the same physical setup.

Since the arm can physically reach every corner of the workspace, it can serve as its own measurement tool — eliminating the ruler entirely.

## Decision

Replace the ruler-based `06_homography.py` with `09_touch_calibration.py`. The new script:

1. Moves the arm to `SCAN_POSE` and opens the camera feed.
2. The user clicks 4 corners in the camera image (same as before).
3. For each corner, the user drives the arm with keyboard controls (W/S for X, A/D for Y, U/J for Z height, I/K for wrist angle, and bracket keys for step size) until the claw physically touches the corner point.
4. The script reads the arm's IK-frame (x, y) coordinates directly — these are the same coordinates the IK solver uses for motion planning.
5. Computes `cv2.getPerspectiveTransform()` and saves the result to `homography_calibration.json` with an identical JSON schema.

Both scripts produce the same output file and schema, so `VisionBridge` requires no changes.

## Rationale

- **Eliminates the largest error source** — the arm's Dynamixel servos have sub-degree repeatability, so the IK-computed (x, y) at the claw tip is far more accurate than a ruler measurement.
- **Perfect coordinate frame alignment** — the physical coordinates are captured in the exact same frame that `solver.py` uses. There is no opportunity for coordinate frame confusion.
- **Drop-in replacement** — the output JSON schema is identical (`workspace_px`, `workspace_cm`, `homography`, `calibrated_at_scan_pose`, `tolerance`, `calibration_date`). No downstream code changes are needed.
- **Faster for the operator** — no ruler needed, no mental coordinate frame mapping.

## Tradeoff

- Requires the arm to be functional (motors powered, serial connected) during calibration. The old `06_homography.py` only needed the camera.
- The arm must be able to physically reach all 4 workspace corners. If the workspace extends beyond the arm's reach envelope, the touch method cannot be used.

`06_homography.py` is retained in the repo as a fallback for situations where the arm cannot move (e.g., motor failure during initial setup).

## Alternatives Considered

### Keep ruler-based calibration as primary

**Rejected because:** The ruler measurement is the dominant source of homography error. Empirically, replacing it with arm-touch coordinates improved pick accuracy at workspace edges.

### Use ArUco markers for automatic corner detection

Place ArUco markers at the 4 corners and detect them automatically — no clicking or arm movement needed.

**Rejected because:** Requires printing and precisely placing markers. The markers must be removed before operation (they interfere with ball detection). More infrastructure for marginal benefit over touch calibration.

### Use the arm to automatically sweep and detect corners

Drive the arm to predefined corners programmatically rather than having the user manually steer with WASD.

**Rejected because:** The workspace boundary is not known a priori — the user defines it by choosing where to click.

## Extension: Height and Wrist Calibration (2026-04)

The touch calibration was extended to also record **grab height (Z)** and **wrist tilt offset (m4)** at each calibration point. Since the operator is already driving the claw to touch the surface, capturing these values adds zero extra effort.

### What changed

1. **New controls during calibration:** I/K keys adjust wrist tilt (m4 offset) so the operator can level the claw at each point.
2. **New JSON fields:** `homography_calibration.json` now includes `height_calibration` (array of `{x, y, z}`) and `wrist_calibration` (array of `{x, y, m4_offset}`).
3. **Runtime interpolation:** [`compute_grab_height(x, y)`](../../src/config/arm.py) and [`compute_wrist_correction(x, y)`](../../src/config/arm.py) linearly interpolate over the calibration arrays to determine the correct Z and m4 values for any given ball position. When no touch calibration data exists, they fall back to formula-based defaults (`GRAB_HEIGHT` constant and 0.0 respectively).
4. **`skip_sag` in IK solver:** [`solve()`](../../src/ik/solver.py) accepts a `skip_sag` parameter. When touch calibration data is available, grab moves use `skip_sag=True` because the touch-calibrated Z already includes real arm droop — applying sag compensation on top would double-compensate.

### Why this supersedes sag calibration for grabs

Sag calibration (Step 03) fits a global linear or quadratic model of droop vs. reach distance. This model is approximate: it assumes droop depends only on reach and ignores the effect of base angle, load, and mechanical variation across the workspace. In contrast, touch calibration captures the *actual* surface height at specific workspace positions — it implicitly accounts for sag, table tilt, and any other height variation. For grab moves, the touch-calibrated Z is therefore more accurate than the sag model.

Sag calibration is **not removed** — it remains active for all non-grab moves (scanning, binning, clearance) where the arm moves to heights above the surface and touch calibration does not apply.
