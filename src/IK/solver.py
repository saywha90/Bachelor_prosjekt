"""
solver.py
Geometric Inverse Kinematics solver for a 4-DOF robotic arm.

Hardware:
    Motor 1 (ID 1) – Base Pan      – XM430
    Motor 2 (ID 2) – Shoulder Tilt – XM540
    Motor 3 (ID 3) – Elbow Tilt    – XM430
    Motor 4 (ID 4) – Wrist Tilt    – XL430
    Motor 5 (ID 5) – Claw          – XL430

All Dynamixel motors: 0-4095 steps → 0°-360°.
Centre (straight ahead / straight up) = 2048 = 180°.

Dependencies: numpy

Author: Bachelor Project 2026 – Autonomia
"""

import json
import logging
import math
from typing import Optional

logger = logging.getLogger(__name__)


class ArmIK:
    """Geometric IK for a 4-DOF pick-and-place arm (+ claw motor)."""

    # ── Claw default position ──────────────────────────────────────────
    CLAW_OPEN: int = 2048   # centre / open position for the gripper

    # ── Link lengths (cm) ──────────────────────────────────────────────
    # Measured from the real robot on 2026-04-21
    L1: float = 25.5   # Shoulder → Elbow
    L2: float = 23.0   # Elbow   → Wrist pivot
    L3: float = 20.5   # Wrist pivot → Claw tip (updated from 16.5 on 2026-04-25)

    # ── Dynamixel constants ────────────────────────────────────────────
    STEPS_PER_REV: int = 4096
    STEP_CENTRE: int = 2048        # 180° = "neutral" (used by motors 2-5)
    M1_CENTRE: int = 2048          # Center for motor 1 (base pan) — same as Dynamixel centre (2048 = straight ahead)
    DEG_PER_STEP: float = 360.0 / 4096.0
    RAD_PER_STEP: float = (2.0 * math.pi) / 4096.0

    # ── Sag / droop compensation ───────────────────────────────────────
    #   The further the arm reaches on the XY plane, the more it droops
    #   under its own weight.  We artificially raise the target Z to
    #   counteract this.
    #
    #   z_correction = horizontal_reach * z_offset_multiplier
    #
    #   Tune this value empirically on your physical arm.
    z_offset_multiplier: float = 0.14   # Increased from 0.04 to compensate for ~6cm sag at far reach
    z_offset_quadratic: float = 0.0     # quadratic sag coefficient (reach^2 term)
    z_offset_constant: float = 0.0      # constant vertical offset (intercept)
    sag_model: str = "linear"           # "linear" or "quadratic"

    # ── Shoulder height above the workspace plane (cm) ─────────────────
    #   If the shoulder joint is elevated above the surface the claw
    #   picks from, set this so the Z math references the shoulder as
    #   origin.  Set to 0 if your coordinate frame already accounts for
    #   this.
    shoulder_height: float = 11.0

    # ── Floor / hover constraint (cm) ─────────────────────────────────
    #   Minimum allowed Z for the claw tip.  Set to 12.0 so the arm
    #   can reach down to grab objects near the desk surface.
    Z_MIN: float = 1.5

    # ── Joint limits (Dynamixel steps) ────────────────────────────────
    #   Safe operating ranges for each motor to prevent overload errors.
    #   If the IK solution falls outside these limits, the motor would
    #   hit a physical stop or overload trying to reach the position,
    #   causing a latched hardware error (red blinking LED).
    #
    #   Tune these based on your physical arm's actual range of motion.
    JOINT_LIMITS = {
        "m1": (0, 4095),       # Base pan: full range
        "m2": (600, 3500),     # Shoulder: avoid extreme up/down
        "m3": (600, 3500),     # Elbow: avoid extreme fold-back
        "m4": (600, 3500),     # Wrist: avoid extreme tilt
        "m5": (0, 4095),       # Claw: full range
    }

    def __init__(
        self,
        l1: Optional[float] = None,
        l2: Optional[float] = None,
        l3: Optional[float] = None,
        z_offset_multiplier: Optional[float] = None,
        z_offset_quadratic: Optional[float] = None,
        z_offset_constant: Optional[float] = None,
        sag_model: Optional[str] = None,
        shoulder_height: Optional[float] = None,
    ) -> None:
        if l1 is not None:
            self.L1 = l1
        if l2 is not None:
            self.L2 = l2
        if l3 is not None:
            self.L3 = l3
        if z_offset_multiplier is not None:
            self.z_offset_multiplier = z_offset_multiplier
        if z_offset_quadratic is not None:
            self.z_offset_quadratic = z_offset_quadratic
        if z_offset_constant is not None:
            self.z_offset_constant = z_offset_constant
        if sag_model is not None:
            self.sag_model = sag_model
        if shoulder_height is not None:
            self.shoulder_height = shoulder_height

        # Auto-load sag calibration if file exists and no explicit overrides given
        if z_offset_multiplier is None and z_offset_quadratic is None and z_offset_constant is None:
            self._load_sag_calibration()

    def _load_sag_calibration(self) -> None:
        """Load sag compensation coefficients from calibration JSON if available."""
        import os
        cal_path = os.path.join(os.path.dirname(__file__), "sag_calibration.json")
        if not os.path.exists(cal_path):
            return  # no calibration file, keep defaults
        try:
            with open(cal_path, "r") as f:
                cal = json.load(f)
            model = cal.get("recommended_model", "linear")
            if model == "quadratic" and "quadratic" in cal:
                self.sag_model = "quadratic"
                self.z_offset_quadratic = cal["quadratic"]["a"]
                self.z_offset_multiplier = cal["quadratic"]["b"]
                self.z_offset_constant = cal["quadratic"].get("c", 0.0)
            elif "linear" in cal:
                self.sag_model = "linear"
                self.z_offset_multiplier = cal["linear"]["z_offset_multiplier"]
                self.z_offset_constant = cal["linear"].get("intercept", 0.0)
            logger.info(
                "[ArmIK] Loaded sag calibration: model=%s, mult=%.4f, quad=%.6f, const=%.4f",
                self.sag_model, self.z_offset_multiplier, self.z_offset_quadratic, self.z_offset_constant,
            )
        except (json.JSONDecodeError, KeyError, TypeError) as e:
            logger.warning("[ArmIK] Failed to load sag calibration: %s", e)

    # ──────────────────────────────────────────────────────────────────
    #  Utility helpers
    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _clamp(value: float, lo: float = -1.0, hi: float = 1.0) -> float:
        """Clamp *value* to [lo, hi] to avoid domain errors in acos."""
        return max(lo, min(hi, value))

    def _rad_to_steps(self, radians: float) -> int:
        """Convert an angle in radians to Dynamixel steps (0-4095).

        Convention: 0 rad → step 2048 (centre).  Positive angle adds steps.
        """
        steps = int(round(self.STEP_CENTRE + radians / self.RAD_PER_STEP))
        return max(0, min(self.STEPS_PER_REV - 1, steps))

    # ──────────────────────────────────────────────────────────────────
    #  Core IK solver
    # ──────────────────────────────────────────────────────────────────
    def solve(self, x: float, y: float, z: float) -> dict:
        """Compute motor step positions for the given (x, y, z) target.

        Parameters
        ----------
        x, y, z : float
            Target coordinates in centimetres, where *z* points up.

        Returns
        -------
        dict
            ``{"m1": int, "m2": int, "m3": int, "m4": int, "m5": int}``
            Dynamixel step values (0-4095) for each motor.

        Raises
        ------
        ValueError
            If the target is unreachable.
        """

        # ── 0. Hover / floor-collision prevention ──────────────────────
        #   The claw tip must stay at least Z_MIN (2 cm) above the desk.
        #   Silently clamp the target z upward so the arm hovers safely.
        if z < self.Z_MIN:
            z = self.Z_MIN

        # ── 1. End-effector offset ────────────────────────────────────
        #   Add L3 to Z so the wrist hovers above the object and the
        #   claw (pointing straight down) reaches the target.
        z_ik = z + self.L3

        # ── 2. Sag / droop compensation (linear or quadratic model) ──
        horiz_reach = math.sqrt(x ** 2 + y ** 2)
        if self.sag_model == "quadratic" and self.z_offset_quadratic != 0.0:
            z_ik += (horiz_reach ** 2) * self.z_offset_quadratic + horiz_reach * self.z_offset_multiplier + self.z_offset_constant
        else:
            z_ik += horiz_reach * self.z_offset_multiplier + self.z_offset_constant

        # ── 3. Account for shoulder height ────────────────────────────
        z_ik -= self.shoulder_height

        # ── 4. Base angle (Motor 1) ───────────────────────────────────
        theta_base = math.atan2(y, x)   # radians

        # ── 5. Planar distance to target (in the arm's 2-D plane) ─────
        r = horiz_reach              # horizontal distance
        d = math.sqrt(r ** 2 + z_ik ** 2)  # straight-line distance

        # Reachability check
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)

        if d > max_reach:
            # Target is slightly too far — scale the horizontal reach
            # inward so the arm extends to its physical limit instead
            # of crashing.  The base angle is preserved, so the arm
            # still points at the correct target direction.
            overshoot = d - max_reach
            logger.warning(
                "[IK] Target (%.1f, %.1f, %.1f) is %.1f cm beyond max reach (%.1f cm) — clamping",
                x, y, z, overshoot, max_reach,
            )
            scale = (max_reach * 0.99) / d   # 0.99 to stay just inside
            r *= scale
            z_ik *= scale
            d = math.sqrt(r ** 2 + z_ik ** 2)
            # Update x, y to match the clamped reach (keep angle)
            if horiz_reach > 0:
                x = x * (r / horiz_reach)
                y = y * (r / horiz_reach)
                horiz_reach = r

        if d < min_reach:
            raise ValueError(
                f"Target ({x}, {y}, {z}) is too close.  "
                f"Planar distance {d:.2f} cm is less than min reach "
                f"{min_reach:.2f} cm."
            )

        # ── 6. Law of Cosines – elbow angle ──────────────────────────
        cos_elbow = (self.L1 ** 2 + self.L2 ** 2 - d ** 2) / (
            2 * self.L1 * self.L2
        )
        cos_elbow = self._clamp(cos_elbow)
        # Interior elbow angle (π when fully extended)
        elbow_interior = math.acos(cos_elbow)
        # Elbow servo angle: 0 = fully folded, π = straight
        # We define positive elbow deflection as "opening up".
        theta_elbow = math.pi - elbow_interior

        # ── 7. Law of Cosines – shoulder angle ───────────────────────
        cos_alpha = (self.L1 ** 2 + d ** 2 - self.L2 ** 2) / (
            2 * self.L1 * d
        )
        cos_alpha = self._clamp(cos_alpha)
        alpha = math.acos(cos_alpha)

        # Angle of the line from shoulder to target, measured from
        # the horizontal plane.
        phi = math.atan2(z_ik, r)

        # Shoulder servo angle relative to horizontal
        theta_shoulder = phi + alpha

        # ── 8. Wrist compensation for vertical end-effector ──────────
        #   We want the claw to point straight down (−Z).  The total
        #   tilt of the arm chain is (shoulder − elbow).  The wrist
        #   must add the remaining rotation to reach −π/2 from
        #   horizontal.
        #
        #   Arm tilt from horizontal = theta_shoulder - theta_elbow
        #   Desired total = -π/2 (pointing down from horizontal)
        #   wrist = -π/2 - (theta_shoulder - theta_elbow)
        theta_wrist = (-math.pi / 2.0) - (theta_shoulder - theta_elbow)

        # ── 9. Convert to Dynamixel steps ────────────────────────────
        # Motor 1 uses M1_CENTRE (2048) — Dynamixel centre = straight ahead.
        m1 = max(0, min(self.STEPS_PER_REV - 1,
                        int(round(self.M1_CENTRE + theta_base / self.RAD_PER_STEP))))
        # m2=2048 is upper arm VERTICAL (straight up), not horizontal.
        # Subtract pi/2 to convert from "elevation above horizontal" (IK convention)
        # to "rotation from vertical" (motor convention).
        m2 = self._rad_to_steps(theta_shoulder - math.pi / 2)
        m3 = self._rad_to_steps(-theta_elbow)  # Elbow requires negation for correct direction
        m4 = self._rad_to_steps(theta_wrist)   # NOT negated — real hardware wrist tilts opposite to simulator

        # ── 10. Enforce joint limits to prevent overload errors ────────
        #    If a computed position exceeds the safe range, clamp it and
        #    warn.  This prevents the motor from hitting physical stops
        #    which causes hardware errors (red blinking LED).
        result = {"m1": m1, "m2": m2, "m3": m3, "m4": m4, "m5": self.CLAW_OPEN}
        for key, val in result.items():
            lo, hi = self.JOINT_LIMITS[key]
            if val < lo or val > hi:
                clamped = max(lo, min(hi, val))
                logger.warning(
                    "[IK] %s=%d outside safe limits [%d, %d] — clamped to %d "
                    "(target %.1f, %.1f, %.1f)",
                    key, val, lo, hi, clamped, x, y, z,
                )
                result[key] = clamped

        return result

    # ──────────────────────────────────────────────────────────────────
    #  Two-step servoing
    # ──────────────────────────────────────────────────────────────────
    def calculate_partial_move(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        percentage: float = 0.80,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        origin_z: float = 0.0,
    ) -> dict:
        """Calculate an IK solution for a *partial* move toward the target.

        This enables **visual servoing**: move 80 % of the way, take a
        new picture, re-compute, then move the final 20 %.

        Parameters
        ----------
        target_x, target_y, target_z : float
            Final target coordinates (cm).
        percentage : float
            Fraction of the distance to travel (0.0 – 1.0).  Default 0.80.
        origin_x, origin_y, origin_z : float
            Current end-effector position (cm).  Defaults to the origin,
            which is appropriate if the arm starts from its home position.

        Returns
        -------
        dict
            Motor step positions for the intermediate point.
        """
        if not 0.0 <= percentage <= 1.0:
            raise ValueError("percentage must be between 0.0 and 1.0")

        # Linear interpolation in Cartesian space
        ix = origin_x + (target_x - origin_x) * percentage
        iy = origin_y + (target_y - origin_y) * percentage
        iz = origin_z + (target_z - origin_z) * percentage

        return self.solve(ix, iy, iz)

    # ──────────────────────────────────────────────────────────────────
    #  JSON output helpers
    # ──────────────────────────────────────────────────────────────────
    def solve_to_json(self, x: float, y: float, z: float) -> str:
        """Return the IK solution as a JSON string ready for the OpenRB."""
        return json.dumps(self.solve(x, y, z))

    def partial_move_to_json(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        percentage: float = 0.80,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        origin_z: float = 0.0,
    ) -> str:
        """Return the partial-move IK solution as a JSON string."""
        return json.dumps(
            self.calculate_partial_move(
                target_x, target_y, target_z, percentage,
                origin_x, origin_y, origin_z,
            )
        )


# ──────────────────────────────────────────────────────────────────────
#  Quick self-test / demo
# ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    arm = ArmIK()

    # Example target: 20 cm in front, 10 cm to the right, on the surface
    target = (20.0, 10.0, 0.0)

    print("=== Arm IK Solver – Self-Test ===\n")
    print(f"Link lengths  : L1={arm.L1} cm, L2={arm.L2} cm, L3={arm.L3} cm")
    print(f"Sag multiplier: {arm.z_offset_multiplier}")
    print(f"Target        : x={target[0]}, y={target[1]}, z={target[2]} cm\n")

    # Full move
    result = arm.solve(*target)
    print(f"Full move steps : {result}")
    print(f"Full move JSON  : {json.dumps(result)}\n")

    # Two-step servoing: 80% then 100%
    partial = arm.calculate_partial_move(*target, percentage=0.80)
    print(f"80 %% move steps: {partial}")
    print(f"80 %% move JSON : {json.dumps(partial)}\n")

    final = arm.calculate_partial_move(*target, percentage=1.0)
    print(f"100 %% move steps: {final}")
    print(f"100 %% move JSON : {json.dumps(final)}")
