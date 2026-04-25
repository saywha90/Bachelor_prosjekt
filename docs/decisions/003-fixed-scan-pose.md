# ADR-003: Fixed Scan Pose over Continuous Visual Servoing

## Status

Accepted

## Date

2026-04-25

## Context

The camera is now wrist-mounted on the arm; it moves with motors 1–4. The homography from pixel→cm coordinates is only valid at a fixed joint configuration. Continuous visual servoing would require either full hand-eye calibration (computing the camera-to-world transform at every arm pose using forward kinematics) or real-time homography updates.

The previous approach (two-step visual servoing with an 80% coarse move followed by a 20% fine correction) was designed for a fixed pillar-mounted camera where the homography remained valid regardless of arm position. With the wrist-mounted camera, this approach is no longer viable because the camera view changes as the arm moves.

## Decision

Use a fixed `SCAN_POSE` for all vision operations. The arm always moves to this known joint configuration before running the detector. The homography is calibrated once at this pose and stored in `homography_calibration.json`.

The state machine enforces this by transitioning through `MOVE_TO_SCAN_POSE` before every `SCANNING` state, and again after `DROPPING` before the next scan cycle. The flow is: `HOME → MOVE_TO_SCAN_POSE → SCANNING → APPROACHING → GRABBING → SORTING → DROPPING → MOVE_TO_SCAN_POSE → ...`

## Rationale

- **Avoids implementing forward kinematics** — computing the camera-to-world transform at every arm pose is complex and error-prone for a 4-DOF arm with sag compensation
- **Avoids self-occlusion during approach** — the claw enters the camera FOV when the arm is close to a ball, making mid-approach visual correction unreliable
- **Simple to calibrate** — tuning `SCAN_POSE` takes ~5–10 minutes (see [calibration Step 02c](../calibration.md)), and the homography calibration at that pose takes another ~5–10 minutes
- **High homography accuracy** when the pose is repeatable — Dynamixel servos have sub-degree repeatability, so returning to `SCAN_POSE` produces a consistent camera view
- **Matches the project's philosophy** of working solutions over theoretical perfection — a fixed scan pose is simple, reliable, and sufficient for the current workspace size

## Tradeoff

No closed-loop visual correction during approach. Final pick accuracy depends entirely on:

1. **Initial detection accuracy** — how precisely the ball centre is located in the camera frame
2. **Homography accuracy** — how well the pixel→cm transform maps to the physical workspace
3. **IK solver accuracy** — how precisely the geometric IK computes motor positions for a given (x, y, z) target
4. **Sag compensation** — how well the droop model corrects for gravity at different reach distances

If any of these degrade, there is no visual feedback loop to correct the error. The system operates open-loop from the moment it leaves `SCAN_POSE` until the pick is complete.

## Alternatives Considered

### Full hand-eye calibration with forward kinematics

Computing the camera-to-world transform at every arm pose would allow continuous visual servoing from any position. This requires:
- Accurate forward kinematics (DH parameters for all 4 joints)
- Camera-to-end-effector extrinsic calibration
- Real-time matrix multiplication for every frame

**Rejected because:** Too complex for a 4-DOF arm with sag compensation that isn't modelled in the DH parameters. The sag model is empirical (linear/quadratic fit) and doesn't translate cleanly into a kinematic chain. Out of scope for the bachelor project.

### Visual servoing during approach

Continuously updating the target position as the arm approaches the ball, using real-time camera feedback to correct the trajectory.

**Rejected because:** The claw occludes the target ball when the arm is close enough to pick. With the wrist-mounted camera looking down, the claw assembly enters the field of view during the final approach phase, making visual correction impossible precisely when it would be most valuable.

### Multiple scan poses for near/far workspace

Using different `SCAN_POSE` configurations depending on whether the target is in the near or far part of the workspace, to improve detection resolution.

**Rejected because:** Unnecessary for the current workspace size (~50 × 30 cm). A single `SCAN_POSE` at 30–40 cm height provides sufficient resolution for 50 mm balls across the entire workspace. The added complexity of pose selection and multiple homography matrices is not justified.

## Future Work (Explicitly Out of Scope)

The following improvements are recognised but intentionally excluded from the current project:

- **True hand-eye calibration with forward kinematics** for continuous camera-to-world transform updates at any arm pose
- **Visual servoing during approach** with closed-loop correction (would require resolving the self-occlusion problem, e.g., with a secondary camera or by mounting the primary camera at a different angle)
- **Multiple scan poses** for near/far workspace regions (e.g., a high pose for overview scanning and a low pose for precise detection of distant balls)
