# ADR-002: Geometric 4-DOF Inverse Kinematics

## Status

Accepted

## Context

The robotic arm has four degrees of freedom (base pan, shoulder tilt, elbow tilt, wrist tilt) plus a claw gripper (motor 5). Given a target position `(x, y, z)` in centimetres relative to the shoulder joint, the system must compute goal positions for each Dynamixel servo (0–4095 steps) so that the claw tip reaches the target with the claw pointing straight down.

The arm's link lengths are:
- L1 = 25.5 cm (shoulder → elbow)
- L2 = 23.0 cm (elbow → wrist pivot)
- L3 = 22.0 cm (wrist pivot → claw tip)

The shoulder joint is elevated 35.0 cm above the workspace surface. The arm needs to reach positions 10–28 cm forward and ±22 cm left/right, picking balls from the table (Z ≈ 6–13 cm) and placing them in bins (Z ≈ 10–12 cm).

## Decision

Use a **closed-form geometric IK solution** as implemented in [`ArmIK.solve()`](../../src/ik/solver.py:157):

1. **Base angle** (motor 1): `θ_base = atan2(y, x)` — decouples the base rotation from the planar arm problem
2. **Planar IK** (motors 2, 3): Law of Cosines on the triangle formed by L1, L2, and the distance `d` from the shoulder to the (adjusted) target point in the arm's vertical plane
3. **Wrist compensation** (motor 4): `θ_wrist = −π/2 − (θ_shoulder − θ_elbow)` — ensures the claw always points straight down
4. **Claw** (motor 5): pass-through, set independently to open (`CLAW_OPEN` = 2745) or close (`CLAW_CLOSED_POS` = 3300)

Additional post-processing steps:
- **End-effector offset:** Z is increased by L3 so the IK solves for the wrist pivot while the claw tip reaches the actual target
- **Sag compensation:** a linear (or quadratic) correction `z_ik += reach × z_offset_multiplier` counteracts gravity droop, with coefficients loaded from [`sag_calibration.json`](../../src/ik/solver.py:115)
- **Joint limits:** each motor's output is clamped to safe ranges (`JOINT_LIMITS` dict: m1 0–4095, m2/m3/m4 600–3500) to prevent hardware overload errors
- **Reach clamping:** targets beyond `L1 + L2` are scaled inward to 99% of maximum reach, preserving the base angle
- **Strict rear-route solving:** production rear placement uses [`ArmIK.solve_strict()`](../../src/ik/solver.py:525) on every route waypoint before movement. Rear routes are fold-over moves: the shoulder/forearm reach behind the robot over the top while base yaw remains guarded, instead of rotating M1 180°.

## Rear-Placement Route Constraint

Rear bins are represented by a strict route schema loaded from
[`bin_calibration.json`](../../src/calibration/bin_calibration.json). The schema
requires shared `front_neutral`, `rear_transfer`, and `rear_return_lift`
waypoints plus per-bin `drop` poses for `RED_BIN` and `BLUE_BIN`.
`rear_return_lift` is the open-claw retreat waypoint used before returning to
`front_neutral`, so the claw clears the rear sorting bin before the arm faces
forward. The real setup does not use a reject bin; no-grip / air-pick recovery
returns to scan/look-again.

The route yaw guard is configured by
[`DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG`](../../src/config/arm.py:227), default
±45°, and may be overridden per calibration file through
`rear_base_yaw_limit_deg`. Strict validation rejects rear route waypoints whose
fold-over base yaw would exceed that configured range. This keeps the base near
front-facing and forces the rear motion to be achieved by the arm links folding
over, which matches the physical clearance constraints of the current setup.

Strict IK validation only proves the route is geometrically reachable and within
configured yaw/joint limits. Final real route `x`/`z` values may still need slow
physical fine-tuning to account for bin wall clearance, payload behaviour, and
small hardware tolerances.

## Rationale

| Criterion | Geometric (Closed-Form) | Numerical Optimisation | ML-based IK |
|---|---|---|---|
| **Exactness** | Exact solution for 4-DOF; no iteration or convergence issues | Iterative — may not converge, or may converge to a local minimum | Approximate — accuracy depends on training data coverage |
| **Speed** | Single evaluation of `atan2`, `acos`, `sqrt` — sub-millisecond | Multiple iterations per solve (1–50 ms depending on tolerance) | Fast inference but requires model loading and GPU for training |
| **Predictability** | Same input always produces the same output, deterministically | May produce different solutions depending on initial guess | Non-deterministic outputs near decision boundaries |
| **Complexity** | ~150 lines of Python (no dependencies beyond `math` and `numpy`) | Requires optimisation framework (scipy, nlopt, etc.) | Requires training infrastructure, data collection, model management |
| **4-DOF suitability** | 4-DOF is analytically solvable — geometric IK is the textbook approach | Numerical methods are needed for 6+ DOF or constrained problems | Overkill for a low-DOF arm with a well-defined geometry |

For a 4-DOF arm where the end-effector orientation is fixed (claw always points down), the problem decomposes cleanly: base rotation is independent, and the remaining 2-DOF planar arm (shoulder + elbow) is solvable in closed form via the Law of Cosines. The wrist angle is then fully determined by the constraint that the claw points downward. This is the standard approach in robotics textbooks for arms of this configuration.

## Alternatives Considered

### Numerical IK (Jacobian / Gradient Descent)
- **Evaluated:** Considered for its generality (would work unchanged if DOF count changed).
- **Rejected because:** Adds unnecessary complexity — convergence tuning, singularity handling, and initial-guess sensitivity are all solved problems for 4-DOF but add engineering overhead. The arm's geometry is fixed; if it changes, the geometric solution is straightforward to re-derive.

### ML-based IK (Neural Network)
- **Evaluated:** Briefly considered for its potential to implicitly learn sag compensation.
- **Rejected because:** Requires collecting thousands of (target, joint-angle) pairs from the physical arm; accuracy near workspace boundaries would be poor without dense sampling; the explicit sag compensation model (`z_offset_multiplier`) is simpler and more transparent.

### Denavit–Hartenberg (DH) Parameter Approach
- **Evaluated:** The standard systematic method for deriving forward/inverse kinematics.
- **Rejected because:** For this specific 4-DOF arm with a fixed end-effector orientation, the DH parameterisation adds notational overhead without producing a simpler solution. The direct geometric derivation is more readable and easier to verify against the physical arm.

## Consequences

### Positive

- **Sub-millisecond solve time** — IK computation is never a bottleneck; the system is limited by serial communication (115200 baud) and motor settling time (1.5 s)
- **Exact solutions** within the reachable workspace — no iterative convergence issues or approximation errors
- **Transparent debugging:** every intermediate angle (`theta_shoulder`, `theta_elbow`, `theta_wrist`) is printed in the debug output, making it easy to trace issues to specific joints
- **Simple sag compensation model:** the post-processing `z_offset_multiplier` is calibrated empirically with a 5-point measurement procedure ([`03_sag.py`](../../src/calibration/03_sag.py)) and auto-loaded from JSON
- **Joint limit enforcement** prevents motor overload errors (red blinking LED) that previously required physical power cycling

### Negative

- **Specific to this arm geometry:** the solution assumes a planar 2-link arm (shoulder + elbow) with an independent base rotation and a wrist that compensates to point straight down. Changing the arm's kinematic structure (e.g., adding a 5th articulated joint, or changing the wrist to allow arbitrary orientation) would require re-deriving the IK equations
- **Sag compensation is a post-processing hack:** the arm's flexibility under load is modelled as a simple function of horizontal reach, not a full elastic model. This is sufficient for the current arm and payloads (50 mm balls, ~20 g) but would not scale to heavier loads or longer links
- **No collision avoidance:** the solver does not model the arm's own geometry or obstacles in the workspace. Joint limits prevent self-collision at extreme positions, but the arm could theoretically collide with bins or the camera pillar at certain configurations
