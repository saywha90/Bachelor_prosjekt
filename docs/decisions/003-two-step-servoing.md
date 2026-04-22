# ADR-003: Two-Step Visual Servoing Approach

## Status

Accepted (infrastructure implemented; fine correction phase currently disabled for stability)

## Context

The arm must accurately position the claw directly above a ball to execute a reliable pick. Several sources of error accumulate between detection and motor execution:

1. **Homography imprecision:** The 4-point perspective transform assumes a perfectly flat workspace and precise corner measurements. In practice, measurement errors of ±2–3 mm per corner propagate to ±5 mm positional error across the workspace.
2. **Mechanical play:** The Dynamixel servos have backlash of approximately 0.09° (XM430/XM540), which translates to ~0.5–1.0 mm positional error at the claw tip at full extension.
3. **Sag compensation residual:** The linear (or quadratic) gravity-droop model corrects for the dominant effect but leaves a residual error of ~1–3 mm depending on arm configuration.
4. **Camera-to-shoulder offset:** Even with shoulder-relative homography calibration, a small systematic offset may remain.

A single open-loop approach (detect → compute → move to 100%) would accumulate all these errors, potentially missing the ball. The ball diameter is 50 mm, and the claw opening is tuned tightly to grip it — a miss of 10+ mm in any direction causes a failed pick.

## Decision

Implement a **two-step visual servoing strategy** in [`ArmIK.calculate_partial_move()`](../../src/ik/solver.py:317) and the [`APPROACHING`](../../src/main.py:317) state of the state machine:

1. **Coarse approach (80%):** Move 80% of the distance from HOME to the target position at `APPROACH_HEIGHT` (24.0 cm). This uses [`send_partial()`](../../src/main.py:128) which calls `ArmIK.calculate_partial_move(target, percentage=0.80)` — a Cartesian-space linear interpolation from the origin to 80% of the target.

2. **Fine correction (20%):** From the 80% position, the system is designed to call [`VisionBridge.refine_detection()`](../../src/ik/vision_bridge.py:564) to take a fresh image, re-detect the ball from a closer viewpoint with reduced parallax error, and move to the updated position. This method captures 3 frames, filters by colour, and returns the corrected coordinates.

3. **Final descent:** Move to the full target position at `GRAB_HEIGHT` (13.0 cm) using [`send_command()`](../../src/main.py:97) with the original (or corrected) coordinates.

**Current status:** The fine correction phase (step 2) is **disabled** in the production code (`main.py` line 331–334) with a comment: *"Skipping correction to maintain grab alignment stability."* The infrastructure (`refine_detection()`, `calculate_partial_move()`) remains fully implemented and tested. The decision to disable it was made empirically — the initial homography calibration proved accurate enough, and re-detecting during approach occasionally produced a slightly different centroid (due to the arm partially occluding the camera's view), which caused more harm than good.

## Rationale

The two-step approach addresses a fundamental trade-off in vision-guided manipulation:

- **Far-field detection** (from HOME, ~40 cm away) sees the full workspace and detects all balls, but has lower positional accuracy due to pixel-to-cm resolution limits and camera obliqueness.
- **Near-field correction** (from the 80% position, ~8 cm above the ball) would have higher pixel-to-cm resolution but risks arm occlusion and reduced field of view.

The partial-move mechanism (`calculate_partial_move()`) interpolates linearly in Cartesian space:
```python
ix = origin_x + (target_x - origin_x) * percentage
iy = origin_y + (target_y - origin_y) * percentage
iz = origin_z + (target_z - origin_z) * percentage
```

This produces a smooth, predictable intermediate position that the IK solver handles identically to a full move. The percentage parameter (0.0–1.0) is validated with a `ValueError` guard.

Even with the fine correction disabled, the two-step approach provides a structural benefit: the arm first approaches at `APPROACH_HEIGHT` (24.0 cm) to avoid collisions, then descends to `GRAB_HEIGHT` (13.0 cm) — separating the XY approach from the Z descent.

## Alternatives Considered

### Single-Step Open-Loop Move
- **Evaluated:** Move directly from HOME to the grab position in one command.
- **Rejected because:** Combines horizontal traverse and vertical descent into one motion, risking collision with balls or bin walls. The two-step approach (approach high → descend) is inherently safer.

### Continuous Visual Servoing (Closed-Loop)
- **Evaluated:** Continuously capture frames and adjust motor positions in a feedback loop until the claw is centred over the ball.
- **Rejected because:** The serial communication latency (JSON serialise + transmit + ACK at 115200 baud) combined with the motor settling time (1.5 s) makes tight feedback loops impractical. Each command-response cycle takes ~1.6 s minimum, so a continuous servoing loop would be very slow (< 1 Hz update rate). The system was designed for offline computation + open-loop execution.

### Look-Then-Move with Error Threshold
- **Evaluated:** Detect, move, re-detect, and iterate until error < threshold.
- **Partially implemented:** The `refine_detection()` method enables this pattern. In practice, a single detection + single move was sufficient — adding iteration increased cycle time without measurably improving pick success rate.

## Consequences

### Positive

- **Structural safety:** The two-height approach (APPROACH_HEIGHT → GRAB_HEIGHT) separates horizontal traverse from vertical descent, preventing collisions during approach
- **Correction infrastructure ready:** `refine_detection()` and `calculate_partial_move()` are fully implemented and can be re-enabled with a single code change if calibration accuracy degrades
- **Testable in simulation:** `calculate_partial_move()` works with `MockSerial` and the 3-D visualiser, allowing verification of partial-move trajectories without hardware
- **Configurable split:** The 80/20 split is a parameter (`percentage=0.80`), not hardcoded logic — it can be adjusted to 70/30 or 90/10 based on empirical results

### Negative

- **Slightly slower cycle time:** The two-step approach adds one extra movement command (80% approach + settle time) compared to a direct move — approximately 1.5 s additional per cycle
- **Correction currently disabled:** The vision correction phase is not active in production, meaning the system relies entirely on the quality of the initial homography calibration. If the camera is bumped or lighting changes significantly, picks may fail until recalibration
- **Arm occlusion risk:** When the arm is at the 80% position, it may partially occlude the camera's view of the target ball — this was the primary reason the correction phase was disabled. A camera mounted directly above the workspace (rather than at an angle from a pillar) would mitigate this issue
