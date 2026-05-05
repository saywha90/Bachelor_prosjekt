"""
solver.py
Geometric Inverse Kinematics solver for a 4-DOF robotic arm.

Hardware:
    Motor 1 (ID 1) – Base Pan      – XM430
    Motor 2 (ID 2) – Shoulder Tilt – XM540
    Motor 3 (ID 3) – Elbow Tilt    – XM430
    Motor 4 (ID 4) – Wrist Tilt    – XL430
    Motor 5 (ID 5) – Claw          – XM430

All Dynamixel motors: 0-4095 steps → 0°-360°.
Centre (straight ahead / straight up) = 2048 = 180°.

Dependencies: numpy

Author: Bachelor Project 2026 – Autonomia
"""

from __future__ import annotations

import json
import logging
import math
import os
from typing import Any

from config.arm import (
    DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG,
    MAX_REACH_PITCH,
    CLAW_OPEN_POS,
)

logger = logging.getLogger(__name__)


class ArmIK:
    """Geometric IK for a 4-DOF pick-and-place arm (+ claw motor)."""

    STRICT_INTENTS = {"pickup", "carry", "rear_place"}
    REAR_FOLD_BACK_MIN_SHOULDER_DEG: float = 90.0

    # ── Claw default position ──────────────────────────────────────────
    CLAW_OPEN: int = CLAW_OPEN_POS   # imported from config.arm

    # ── Link lengths (cm) ──────────────────────────────────────────────
    # Measured from the real robot on 2026-04-21
    L1: float = 25.5   # Shoulder → Elbow
    L2: float = 23.0   # Elbow   → Wrist pivot
    L3: float = 22.0   # Wrist pivot → Claw tip (updated from 20.5 on 2026-04-27)

    # ── Dynamixel constants ────────────────────────────────────────────
    STEPS_PER_REV: int = 4096
    STEP_CENTRE: int = 2048        # 180° = "neutral" (used by motors 2, 3, 5)
    M1_CENTRE: int = 2048          # Center for motor 1 (base pan) — same as Dynamixel centre (2048 = straight ahead)
    M4_CENTRE: int = 1911          # Center for motor 4 (wrist tilt) — 3D-printed mount shifts mechanical centre from 2048 to 1911
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
    z_offset_multiplier: float = 0.18   # Increased to 0.18 on 2026-04-25 (lifts arm away from desk)
    z_offset_quadratic: float = 0.0     # quadratic sag coefficient (reach^2 term)
    z_offset_constant: float = 0.0      # constant vertical offset (intercept)
    sag_model: str = "linear"           # "linear" or "quadratic"

    # ── Shoulder height above the workspace plane (cm) ─────────────────
    #   If the shoulder joint is elevated above the surface the claw
    #   picks from, set this so the Z math references the shoulder as
    #   origin.  Set to 0 if your coordinate frame already accounts for
    #   this.
    shoulder_height: float = 35.0   # Measured 2026-04-29 (was 11.0 — claw was 20 cm above ground at z=-1.7)

    # ── Floor / hover constraint (cm) ─────────────────────────────────
    #   Minimum allowed Z for the claw tip.  Set to -2.0 so the arm
    #   can reach below the desk surface for touch calibration.
    Z_MIN: float = -2.0

    # ── Sag correction safety clamps (cm) ─────────────────────────────
    SAG_CORRECTION_MIN: float = -15.0  # max downward correction (cm)
    SAG_CORRECTION_MAX: float = 20.0   # max upward correction (cm)

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
        "m4": (500, 3500),     # Wrist: avoid extreme tilt (lowered from 600 on 2026-04-27 — Ball #1 needed ~530)
        "m5": (0, 4095),       # Claw: full range
    }

    def __init__(
        self,
        l1: float | None = None,
        l2: float | None = None,
        l3: float | None = None,
        z_offset_multiplier: float | None = None,
        z_offset_quadratic: float | None = None,
        z_offset_constant: float | None = None,
        sag_model: str | None = None,
        shoulder_height: float | None = None,
        rear_base_yaw_limit_deg: float | None = None,
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
        if rear_base_yaw_limit_deg is None:
            self.rear_base_yaw_limit_deg = float(DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG)
        else:
            self.rear_base_yaw_limit_deg = self._parse_rear_base_yaw_limit(rear_base_yaw_limit_deg)

        # Auto-load sag calibration if file exists and no explicit overrides given
        if z_offset_multiplier is None and z_offset_quadratic is None and z_offset_constant is None:
            self._load_sag_calibration()

    def _load_sag_calibration(self) -> None:
        """Load sag compensation coefficients from calibration JSON if available."""
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
                lin = cal["linear"]
                # Support both key names: "slope" (new) and "z_offset_multiplier" (legacy)
                self.z_offset_multiplier = lin.get("slope", lin.get("z_offset_multiplier", 0.0))
                self.z_offset_constant = lin.get("intercept", 0.0)
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

    def _rad_to_steps(self, radians: float, centre: int | None = None) -> int:
        """Convert an angle in radians to Dynamixel steps (0-4095).

        Convention: 0 rad → *centre* step (default STEP_CENTRE = 2048).
        Positive angle adds steps.

        Parameters
        ----------
        radians : float
            Angle in radians.
        centre : int or None
            Step value corresponding to 0 rad.  Defaults to
            ``STEP_CENTRE`` (2048) for most motors.  Pass
            ``M4_CENTRE`` (1911) for the wrist tilt motor whose
            3-D-printed mount shifts the mechanical neutral.
        """
        if centre is None:
            centre = self.STEP_CENTRE
        steps = int(round(centre + radians / self.RAD_PER_STEP))
        return max(0, min(self.STEPS_PER_REV - 1, steps))

    def _rad_to_steps_unclamped(self, radians: float, centre: int | None = None) -> int:
        """Convert radians to Dynamixel steps without clamping."""
        if centre is None:
            centre = self.STEP_CENTRE
        return int(round(centre + radians / self.RAD_PER_STEP))

    @staticmethod
    def _normalize_angle_rad(angle: float) -> float:
        """Normalize an angle to [-π, π]."""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def _rear_fold_base_angle(cls, x: float, y: float) -> float:
        """Return the small-yaw base angle for reaching a rear target by folding over."""
        if abs(x) < 1e-9 and abs(y) < 1e-9:
            return 0.0
        return cls._normalize_angle_rad(math.atan2(y, x) + math.pi)

    @staticmethod
    def _parse_rear_base_yaw_limit(raw_limit: Any) -> float:
        """Validate and return a symmetric rear-route base-yaw limit in degrees."""
        try:
            limit = abs(float(raw_limit))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"rear base yaw limit must be numeric, got {raw_limit!r}") from exc
        if not math.isfinite(limit) or limit < 0.0 or limit > 180.0:
            raise ValueError(f"rear base yaw limit {raw_limit!r} must be finite and within [0, 180] degrees")
        return limit

    def _select_rear_fold_solution(
        self,
        *,
        x: float,
        y: float,
        z: float,
        intent: str,
        sag_correction: float,
        min_reach: float,
        max_reach: float,
        m4_offset: int,
        m5: int,
    ) -> tuple[float, float, float, float, float, float, float, float, str]:
        """Select a strict rear fold-over IK branch without spinning the base."""
        target_r = -math.sqrt(x ** 2 + y ** 2)
        theta_pitch = -math.pi / 2.0
        pitch_step = math.radians(1)
        min_shoulder = math.radians(self.REAR_FOLD_BACK_MIN_SHOULDER_DEG)
        m4_offset_rad = int(m4_offset or 0) * self.RAD_PER_STEP

        rejection_reason = "no fold-over branch candidate was generated"
        while True:
            final_theta_pitch = theta_pitch + m4_offset_rad
            wrist_r = target_r - self.L3 * math.cos(final_theta_pitch)
            wrist_z = z - self.L3 * math.sin(final_theta_pitch)
            wrist_z_ik = wrist_z + sag_correction - self.shoulder_height

            wrist_z_floor = self.Z_MIN - self.L3 * math.sin(final_theta_pitch) - self.shoulder_height
            if wrist_z_ik < wrist_z_floor:
                raise ValueError(
                    f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                    f"sag-corrected wrist_z_ik={wrist_z_ik:.2f} would violate floor "
                    f"limit {wrist_z_floor:.2f}; strict path does not clamp"
                )

            d = math.sqrt(wrist_r ** 2 + wrist_z_ik ** 2)
            if min_reach <= d <= max_reach:
                cos_elbow = (self.L1 ** 2 + self.L2 ** 2 - d ** 2) / (2 * self.L1 * self.L2)
                cos_alpha = (self.L1 ** 2 + d ** 2 - self.L2 ** 2) / (2 * self.L1 * d)
                if -1.0000001 <= cos_elbow <= 1.0000001 and -1.0000001 <= cos_alpha <= 1.0000001:
                    cos_elbow = max(-1.0, min(1.0, cos_elbow))
                    cos_alpha = max(-1.0, min(1.0, cos_alpha))
                    elbow_interior = math.acos(cos_elbow)
                    theta_elbow = math.pi - elbow_interior
                    alpha = math.acos(cos_alpha)
                    phi = math.atan2(wrist_z_ik, wrist_r)

                    for branch_sign, branch_name in ((1.0, "fold_back_high"), (-1.0, "fold_back_low")):
                        theta_shoulder = phi + branch_sign * alpha
                        if theta_shoulder < min_shoulder:
                            rejection_reason = (
                                f"candidate shoulder angle {math.degrees(theta_shoulder):.1f}° "
                                f"is below fold-back minimum {self.REAR_FOLD_BACK_MIN_SHOULDER_DEG:.1f}°"
                            )
                            continue

                        link2_angle = theta_shoulder - branch_sign * theta_elbow
                        theta_wrist = self._normalize_angle_rad(theta_pitch - link2_angle)
                        raw_commands = {
                            "m2": self._rad_to_steps_unclamped(theta_shoulder - math.pi / 2),
                            "m3": self._rad_to_steps_unclamped(-branch_sign * theta_elbow),
                            "m4": self._rad_to_steps_unclamped(theta_wrist, centre=self.M4_CENTRE) + m4_offset,
                            "m5": m5,
                        }
                        invalid = []
                        for key, value in raw_commands.items():
                            lo, hi = self.JOINT_LIMITS[key]
                            if value < lo or value > hi:
                                invalid.append(f"{key}={value} outside [{lo}, {hi}]")
                        if invalid:
                            rejection_reason = "; ".join(invalid)
                            continue

                        return (
                            wrist_r,
                            wrist_z,
                            wrist_z_ik,
                            wrist_r,
                            d,
                            theta_pitch,
                            branch_sign,
                            target_r,
                            branch_name,
                        )
                else:
                    rejection_reason = "triangle cosine outside [-1, 1]"
            elif d < min_reach:
                rejection_reason = f"planar distance {d:.2f} cm is less than min reach {min_reach:.2f} cm"
            else:
                rejection_reason = f"planar distance {d:.2f} cm exceeds max reach {max_reach:.2f} cm"

            if theta_pitch >= MAX_REACH_PITCH:
                break
            theta_pitch = min(theta_pitch + pitch_step, MAX_REACH_PITCH)

        raise ValueError(
            f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
            f"target is unreachable by rear fold-over branch within joint limits ({rejection_reason})"
        )

    # ──────────────────────────────────────────────────────────────────
    #  Core IK solver
    # ──────────────────────────────────────────────────────────────────
    def solve(
        self,
        x: float,
        y: float,
        z: float,
        skip_sag: bool = False,
        strict: bool = False,
        m4_offset: int = 0,
    ) -> dict:
        """Compute motor step positions for the given (x, y, z) target.

        Parameters
        ----------
        x, y, z : float
            Target coordinates in centimetres, where *z* points up.
        skip_sag : bool
            If True, skip the internal sag/droop compensation.
        strict : bool
            If True, raise ValueError if the target is out of reach instead of clamping.
        m4_offset : int
            Wrist trim in Dynamixel steps.  The trim changes the final claw
            pitch, so it participates in wrist-position geometry instead of
            being added only after IK.  This keeps limp-mode FK captures
            replayable at the same claw-tip XYZ.

        Returns
        -------
        dict
            ``{"m1": int, "m2": int, "m3": int, "m4": int, "m5": int}``

        Raises
        ------
        ValueError
            If the target is unreachable (and strict is True, or if too close).
        """

        m4_offset = int(m4_offset or 0)
        m4_offset_rad = m4_offset * self.RAD_PER_STEP

        # ── 0. Hover / floor-collision prevention ──────────────────────
        #   The claw tip must stay at least Z_MIN (2 cm) above the desk.
        #   Silently clamp the target z upward so the arm hovers safely.
        if z < self.Z_MIN:
            z = self.Z_MIN

        # ── 1. Dynamic claw pitch for extended reach ──────────────────
        #   Start with claw pointing straight down (−π/2). If the target
        #   is beyond reach, tilt the wrist forward in 1° steps until
        #   the 2-link sub-chain can reach the wrist position, or until
        #   we hit MAX_REACH_PITCH.
        theta_pitch = -math.pi / 2.0  # start: straight down

        # Sag / droop compensation (compute once from original horiz_reach)
        horiz_reach = math.sqrt(x ** 2 + y ** 2)
        if skip_sag:
            sag_correction = 0.0
        else:
            if self.sag_model == "quadratic" and self.z_offset_quadratic != 0.0:
                sag_correction = (horiz_reach ** 2) * self.z_offset_quadratic + horiz_reach * self.z_offset_multiplier + self.z_offset_constant
            else:
                sag_correction = horiz_reach * self.z_offset_multiplier + self.z_offset_constant

            # ── Safety clamp on sag correction ─────────────────────────
            #   Prevent the polynomial model from producing dangerously
            #   large corrections outside the calibrated reach range.
            raw_sag = sag_correction
            sag_correction = max(self.SAG_CORRECTION_MIN, min(sag_correction, self.SAG_CORRECTION_MAX))
            if sag_correction != raw_sag:
                logger.warning(
                    "[IK] Sag correction %.2f cm clamped to [%.1f, %.1f] → %.2f cm "
                    "(horiz_reach=%.1f)",
                    raw_sag, self.SAG_CORRECTION_MIN, self.SAG_CORRECTION_MAX,
                    sag_correction, horiz_reach,
                )

        max_reach = self.L1 + self.L2
        pitch_step = math.radians(1)  # 1° per iteration

        while True:
            # Wrist position derived from final claw pitch. ``m4_offset``
            # changes that final pitch, so it must be included before
            # solving the shoulder/elbow geometry.
            final_theta_pitch = theta_pitch + m4_offset_rad
            wrist_x = horiz_reach - self.L3 * math.cos(final_theta_pitch)
            wrist_z = z - self.L3 * math.sin(final_theta_pitch)

            # Apply sag compensation and shoulder height offset
            wrist_z_ik = wrist_z + sag_correction - self.shoulder_height

            # ── Floor safety clamp ─────────────────────────────────
            #   Ensure the sag-corrected target doesn't drive the claw
            #   below Z_MIN (accounting for shoulder height).
            wrist_z_floor = self.Z_MIN - self.L3 * math.sin(final_theta_pitch) - self.shoulder_height
            if wrist_z_ik < wrist_z_floor:
                logger.warning(
                    "[IK] Sag-corrected wrist_z_ik=%.2f would place claw below Z_MIN=%.1f — "
                    "clamping wrist_z_ik to %.2f (target z=%.1f, sag_correction=%.2f)",
                    wrist_z_ik, self.Z_MIN, wrist_z_floor, z, sag_correction,
                )
                wrist_z_ik = wrist_z_floor

            # Planar reach from shoulder to wrist in the arm's 2D plane.
            # ``wrist_x`` is already a radial distance after projecting the
            # Cartesian target onto the base yaw direction.
            r = wrist_x
            d = math.sqrt(r ** 2 + wrist_z_ik ** 2)

            if d <= max_reach:
                break  # Reachable — use this pitch

            # Not reachable; try tilting forward (increasing pitch toward 0)
            if theta_pitch >= MAX_REACH_PITCH:
                break  # Hit the limit; use current (most-tilted) pitch

            theta_pitch += pitch_step  # tilt forward (from -π/2 toward 0)

        # ── 2. Use the final wrist position for remaining IK ──────────
        z_ik = wrist_z_ik

        # ── 3. Base angle (Motor 1) ───────────────────────────────────
        theta_base = math.atan2(y, x)   # radians

        # ── 4. Planar distance to target (in the arm's 2-D plane) ─────
        # r and d already computed in the pitch loop above

        # Reachability check
        min_reach = abs(self.L1 - self.L2)

        if d > max_reach:
            if strict:
                raise ValueError(f"Target ({x:.1f}, {y:.1f}, {z:.1f}) is unreachable (overshoot: {d - max_reach:.1f} cm)")
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

        # ── 5. Law of Cosines – elbow angle ──────────────────────────
        cos_elbow = (self.L1 ** 2 + self.L2 ** 2 - d ** 2) / (
            2 * self.L1 * self.L2
        )
        cos_elbow = self._clamp(cos_elbow)
        # Interior elbow angle (π when fully extended)
        elbow_interior = math.acos(cos_elbow)
        # Elbow servo angle: 0 = fully folded, π = straight
        # We define positive elbow deflection as "opening up".
        theta_elbow = math.pi - elbow_interior

        # ── 6. Law of Cosines – shoulder angle ───────────────────────
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

        # ── 7. Wrist angle using dynamic pitch ────────────────────────
        #   theta_pitch starts at −π/2 (straight down) and may have been
        #   tilted forward by the pitch loop above.  The wrist servo must
        #   compensate for the arm chain's orientation so the claw reaches
        #   the desired pitch.
        theta_wrist = theta_pitch - (theta_shoulder - theta_elbow)

        # ── 8. Convert to Dynamixel steps ────────────────────────────
        # Motor 1 uses M1_CENTRE (2048) — Dynamixel centre = straight ahead.
        m1 = max(0, min(self.STEPS_PER_REV - 1,
                        int(round(self.M1_CENTRE + theta_base / self.RAD_PER_STEP))))
        # m2=2048 is upper arm VERTICAL (straight up), not horizontal.
        # Subtract pi/2 to convert from "elevation above horizontal" (IK convention)
        # to "rotation from vertical" (motor convention).
        m2 = self._rad_to_steps(theta_shoulder - math.pi / 2)
        m3 = self._rad_to_steps(-theta_elbow)  # Elbow requires negation for correct direction
        m4 = self._rad_to_steps(theta_wrist, centre=self.M4_CENTRE) + m4_offset  # NOT negated — real hardware wrist tilts opposite to simulator

        # ── 9. Enforce joint limits to prevent overload errors ────────
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

    def solve_strict(self, pose: Any, intent: str) -> dict:
        """Strict IK solve for prevalidated production routes.

        Unlike ``solve()``, this path rejects invalid requests instead of
        silently clamping floor height, reach, joint limits, or wrist trim.
        ``intent`` must explicitly be one of ``pickup``, ``carry``, or
        ``rear_place``.  ``pose`` may be a mapping/object with ``x``, ``y``,
        ``z`` and optional ``m4_offset``, ``m5``, and ``skip_sag`` fields, or
        a three-item ``(x, y, z)`` sequence.

        Returns a structured dictionary containing ``commands`` and
        ``validation``.  Raises ``ValueError`` with a clear reason on failure.
        """
        if intent not in self.STRICT_INTENTS:
            raise ValueError(
                f"Invalid strict IK intent {intent!r}; expected one of "
                f"{sorted(self.STRICT_INTENTS)}"
            )

        x, y, z, m4_offset, m5, skip_sag, rear_base_yaw_limit_override = self._parse_strict_pose(pose)
        original_pose = {
            "x": x,
            "y": y,
            "z": z,
            "m4_offset": m4_offset,
            "m5": m5,
            "skip_sag": skip_sag,
        }
        if rear_base_yaw_limit_override is not None:
            original_pose["rear_base_yaw_limit_deg"] = rear_base_yaw_limit_override

        if z < self.Z_MIN:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"z is below Z_MIN={self.Z_MIN:.1f}; strict path does not clamp"
            )

        horiz_reach = math.sqrt(x ** 2 + y ** 2)
        if skip_sag:
            sag_correction = 0.0
        elif self.sag_model == "quadratic" and self.z_offset_quadratic != 0.0:
            sag_correction = (
                (horiz_reach ** 2) * self.z_offset_quadratic
                + horiz_reach * self.z_offset_multiplier
                + self.z_offset_constant
            )
        else:
            sag_correction = horiz_reach * self.z_offset_multiplier + self.z_offset_constant

        if sag_correction < self.SAG_CORRECTION_MIN or sag_correction > self.SAG_CORRECTION_MAX:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"sag correction {sag_correction:.2f} cm outside feasible range "
                f"[{self.SAG_CORRECTION_MIN:.1f}, {self.SAG_CORRECTION_MAX:.1f}]"
            )

        theta_pitch = -math.pi / 2.0
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)
        pitch_step = math.radians(1)
        selected = None
        m4_offset_rad = m4_offset * self.RAD_PER_STEP

        rear_base_yaw_limit_deg = self._parse_rear_base_yaw_limit(
            rear_base_yaw_limit_override
            if rear_base_yaw_limit_override is not None
            else self.rear_base_yaw_limit_deg
        )
        base_yaw_limit_range = (-rear_base_yaw_limit_deg, rear_base_yaw_limit_deg)
        target_r = horiz_reach
        shoulder_branch_sign = 1.0
        ik_branch = "front_forward"

        if intent == "rear_place":
            theta_base = self._rear_fold_base_angle(x, y)
            theta_base_deg = math.degrees(theta_base)
            if theta_base_deg < base_yaw_limit_range[0] - 1e-9 or theta_base_deg > base_yaw_limit_range[1] + 1e-9:
                raise ValueError(
                    f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                    f"rear-route base yaw {theta_base_deg:.1f}° outside configured range "
                    f"[{base_yaw_limit_range[0]:.1f}°, {base_yaw_limit_range[1]:.1f}°]; "
                    f"strict path does not spin the base around"
                )
            selected = self._select_rear_fold_solution(
                x=x,
                y=y,
                z=z,
                intent=intent,
                sag_correction=sag_correction,
                min_reach=min_reach,
                max_reach=max_reach,
                m4_offset=m4_offset,
                m5=m5,
            )
            _wrist_x, _wrist_z, z_ik, r, d, theta_pitch, shoulder_branch_sign, target_r, ik_branch = selected
        else:
            while True:
                final_theta_pitch = theta_pitch + m4_offset_rad
                wrist_x = horiz_reach - self.L3 * math.cos(final_theta_pitch)
                wrist_z = z - self.L3 * math.sin(final_theta_pitch)
                wrist_z_ik = wrist_z + sag_correction - self.shoulder_height

                wrist_z_floor = self.Z_MIN - self.L3 * math.sin(final_theta_pitch) - self.shoulder_height
                if wrist_z_ik < wrist_z_floor:
                    raise ValueError(
                        f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                        f"sag-corrected wrist_z_ik={wrist_z_ik:.2f} would violate floor "
                        f"limit {wrist_z_floor:.2f}; strict path does not clamp"
                    )

                r = wrist_x
                d = math.sqrt(r ** 2 + wrist_z_ik ** 2)
                if d <= max_reach:
                    selected = (wrist_x, wrist_z, wrist_z_ik, r, d, theta_pitch)
                    break

                if theta_pitch >= MAX_REACH_PITCH:
                    break
                theta_pitch = min(theta_pitch + pitch_step, MAX_REACH_PITCH)

            theta_base = math.atan2(y, x)

        if selected is None:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"target is unreachable without projection/clamping"
            )

        if intent != "rear_place":
            _wrist_x, _wrist_z, z_ik, r, d, theta_pitch = selected
        if d < min_reach:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"planar distance {d:.2f} cm is less than min reach {min_reach:.2f} cm"
            )

        cos_elbow = (self.L1 ** 2 + self.L2 ** 2 - d ** 2) / (2 * self.L1 * self.L2)
        if cos_elbow < -1.0000001 or cos_elbow > 1.0000001:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"elbow cosine {cos_elbow:.6f} outside [-1, 1]"
            )
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_interior = math.acos(cos_elbow)
        theta_elbow = math.pi - elbow_interior

        cos_alpha = (self.L1 ** 2 + d ** 2 - self.L2 ** 2) / (2 * self.L1 * d)
        if cos_alpha < -1.0000001 or cos_alpha > 1.0000001:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"shoulder cosine {cos_alpha:.6f} outside [-1, 1]"
            )
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha = math.acos(cos_alpha)

        phi = math.atan2(z_ik, r)
        theta_shoulder = phi + shoulder_branch_sign * alpha
        if intent == "rear_place" and theta_shoulder < math.radians(self.REAR_FOLD_BACK_MIN_SHOULDER_DEG):
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"selected shoulder angle {math.degrees(theta_shoulder):.1f}° is below "
                f"fold-back minimum {self.REAR_FOLD_BACK_MIN_SHOULDER_DEG:.1f}°"
            )
        link2_angle = theta_shoulder - shoulder_branch_sign * theta_elbow
        theta_wrist = theta_pitch - link2_angle
        if intent == "rear_place":
            theta_wrist = self._normalize_angle_rad(theta_wrist)

        m1 = int(round(self.M1_CENTRE + theta_base / self.RAD_PER_STEP))
        m2 = self._rad_to_steps_unclamped(theta_shoulder - math.pi / 2)
        m3 = self._rad_to_steps_unclamped(-shoulder_branch_sign * theta_elbow)
        raw_m4 = self._rad_to_steps_unclamped(theta_wrist, centre=self.M4_CENTRE)
        m4 = raw_m4 + m4_offset

        if raw_m4 < 0 or raw_m4 > self.STEPS_PER_REV - 1:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"raw wrist command m4={raw_m4} outside servo range"
                f"{'; rear fold-over branch rejected' if intent == 'rear_place' else ''}"
            )
        if m4 < 0 or m4 > self.STEPS_PER_REV - 1:
            raise ValueError(
                f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                f"wrist trim/offset infeasible (raw m4={raw_m4}, offset={m4_offset}, final={m4})"
                f"{'; rear fold-over branch rejected' if intent == 'rear_place' else ''}"
            )

        commands = {"m1": m1, "m2": m2, "m3": m3, "m4": m4, "m5": m5}
        joint_validation = {}
        for key, val in commands.items():
            lo, hi = self.JOINT_LIMITS[key]
            ok = lo <= val <= hi
            joint_validation[key] = {"value": val, "min": lo, "max": hi, "ok": ok}
            if not ok:
                raise ValueError(
                    f"Strict IK rejected {intent} pose ({x:.1f}, {y:.1f}, {z:.1f}): "
                    f"joint limit violation {key}={val} outside [{lo}, {hi}]"
                )

        return {
            "commands": commands,
            "validation": {
                "intent": intent,
                "pose": original_pose,
                "horiz_reach_cm": horiz_reach,
                "sag_correction_cm": sag_correction,
                "planar_distance_cm": d,
                "planar_wrist_radius_cm": r,
                "target_radius_cm": target_r,
                "max_reach_cm": max_reach,
                "min_reach_cm": min_reach,
                "ik_branch": ik_branch,
                "theta_base_deg": math.degrees(theta_base),
                "base_yaw_deg": math.degrees(theta_base),
                "base_yaw_range_deg": list(base_yaw_limit_range) if intent == "rear_place" else None,
                "base_yaw_within_range": (
                    base_yaw_limit_range[0] - 1e-9 <= math.degrees(theta_base) <= base_yaw_limit_range[1] + 1e-9
                    if intent == "rear_place" else True
                ),
                "theta_shoulder_deg": math.degrees(theta_shoulder),
                "shoulder_fold_back_min_deg": self.REAR_FOLD_BACK_MIN_SHOULDER_DEG if intent == "rear_place" else None,
                "shoulder_in_fold_back_range": (
                    math.degrees(theta_shoulder) >= self.REAR_FOLD_BACK_MIN_SHOULDER_DEG
                    if intent == "rear_place" else False
                ),
                "theta_pitch_deg": math.degrees(theta_pitch),
                "final_theta_pitch_deg": math.degrees(theta_pitch + m4_offset_rad),
                "raw_m4": raw_m4,
                "m4_offset": m4_offset,
                "joint_limits": joint_validation,
            },
        }

    def _parse_strict_pose(self, pose: Any) -> tuple[float, float, float, int, int, bool, float | None]:
        """Parse strict IK pose input into typed fields."""
        if isinstance(pose, dict):
            getter = pose.get
        elif isinstance(pose, (tuple, list)) and len(pose) == 3:
            getter = {"x": pose[0], "y": pose[1], "z": pose[2]}.get
        else:
            getter = lambda key, default=None: getattr(pose, key, default)

        missing = [key for key in ("x", "y", "z") if getter(key) is None]
        if missing:
            raise ValueError(f"Strict IK pose missing required field(s): {', '.join(missing)}")

        try:
            x = float(getter("x"))
            y = float(getter("y"))
            z = float(getter("z"))
            m4_offset = int(getter("m4_offset", 0) or 0)
            m5 = int(getter("m5", self.CLAW_OPEN) or self.CLAW_OPEN)
            skip_sag = bool(getter("skip_sag", False))
            raw_rear_base_yaw_limit = getter("rear_base_yaw_limit_deg", None)
            rear_base_yaw_limit = (
                None
                if raw_rear_base_yaw_limit is None
                else self._parse_rear_base_yaw_limit(raw_rear_base_yaw_limit)
            )
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Strict IK pose has invalid numeric field: {exc}") from exc

        for key, value in {"x": x, "y": y, "z": z}.items():
            if not math.isfinite(value):
                raise ValueError(f"Strict IK pose field {key} must be finite, got {value!r}")
        if not 0 <= m5 <= self.STEPS_PER_REV - 1:
            raise ValueError(f"Strict IK pose m5={m5} outside servo range [0, {self.STEPS_PER_REV - 1}]")

        return x, y, z, m4_offset, m5, skip_sag, rear_base_yaw_limit

    # ──────────────────────────────────────────────────────────────────
    #  Forward Kinematics
    # ──────────────────────────────────────────────────────────────────
    def forward_kinematics(self, positions: dict) -> dict:
        """Compute end-effector Cartesian position from motor step values.

        This is the reverse of :meth:`solve` — it converts Dynamixel step
        positions back to workspace coordinates using the same geometric
        model and angle conventions.

        Parameters
        ----------
        positions : dict
            Motor positions in Dynamixel steps, e.g.
            ``{"m1": 2048, "m2": 1500, "m3": 2200, "m4": 1800, "m5": 2048}``.
            Keys ``"m1"`` through ``"m4"`` are required; ``"m5"`` (claw) is
            ignored but accepted.

        Returns
        -------
        dict
            ``{"x": float, "y": float, "z": float, "m4_offset": int}``
            where *x*, *y*, *z* are in centimetres (same frame as the IK
            solver) and *m4_offset* is the wrist-tilt motor's deviation
            from its mechanical neutral (``M4_CENTRE``), in steps.
        """

        required = {"m1", "m2", "m3", "m4"}
        missing = required - set(positions.keys())
        if missing:
            raise ValueError(f"Missing motor keys for FK: {missing}")

        m1 = positions["m1"]
        m2 = positions["m2"]
        m3 = positions["m3"]
        m4 = positions["m4"]

        # ── 1. Convert steps → joint angles (exact inverse of solve()) ─
        #   m1 = M1_CENTRE + theta_base / RAD_PER_STEP
        theta_base = (m1 - self.M1_CENTRE) * self.RAD_PER_STEP

        #   m2 = STEP_CENTRE + (theta_shoulder − π/2) / RAD_PER_STEP
        theta_shoulder = (m2 - self.STEP_CENTRE) * self.RAD_PER_STEP + math.pi / 2.0

        #   m3 = STEP_CENTRE + (−theta_elbow) / RAD_PER_STEP
        theta_elbow = -(m3 - self.STEP_CENTRE) * self.RAD_PER_STEP

        #   m4 = M4_CENTRE + theta_wrist / RAD_PER_STEP
        theta_wrist = (m4 - self.M4_CENTRE) * self.RAD_PER_STEP

        # ── 2. FK in the arm's radial–Z plane (shoulder = origin) ──────
        #   The second link leaves the elbow at angle (θ_shoulder − θ_elbow)
        #   from the horizontal, matching the IK triangle geometry.
        link2_angle = theta_shoulder - theta_elbow

        # Wrist pivot position (end of L2)
        r_wrist = self.L1 * math.cos(theta_shoulder) + self.L2 * math.cos(link2_angle)
        z_wrist = self.L1 * math.sin(theta_shoulder) + self.L2 * math.sin(link2_angle)

        # Claw pitch — from IK: theta_wrist = theta_pitch − (θ_shoulder − θ_elbow)
        theta_pitch = theta_wrist + link2_angle

        # Claw-tip position (end of L3)
        r_tip = r_wrist + self.L3 * math.cos(theta_pitch)
        z_tip_ik = z_wrist + self.L3 * math.sin(theta_pitch)

        # ── 3. Convert shoulder-relative frame → workspace frame ───────
        #   IK uses:  z_ik = z − shoulder_height  (ignoring sag)
        #   Therefore: z = z_ik + shoulder_height
        z = z_tip_ik + self.shoulder_height
        x = r_tip * math.cos(theta_base)
        y = r_tip * math.sin(theta_base)

        # ── 4. Wrist diagnostics ──────────────────────────────────────
        m4_offset = m4 - self.M4_CENTRE
        # Replay offset used by IK.  This is intentionally different from
        # ``m4_offset`` above: it is the final claw pitch's deviation from
        # IK's default straight-down pitch, so solving the returned XYZ with
        # this offset preserves the claw-tip location instead of rotating L3
        # after the Cartesian solve.
        replay_m4_offset = int(round((theta_pitch - (-math.pi / 2.0)) / self.RAD_PER_STEP))

        return {
            "x": round(x, 4),
            "y": round(y, 4),
            "z": round(z, 4),
            "m4_offset": m4_offset,
            "replay_m4_offset": replay_m4_offset,
            "theta_pitch_deg": round(math.degrees(theta_pitch), 4),
        }

    # ──────────────────────────────────────────────────────────────────
    #  Partial-move interpolation (utility for tests and demos)
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

        Linearly interpolates in Cartesian space between the origin and
        the target, then solves IK at the intermediate point.

        .. note::
           The production pick-and-place loop (``main.py``) no longer
           uses this method — it moves directly to the grab position in
           a single step (see ADR-003).  This helper is retained for
           tests and manual demo scripts.

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

    # Partial-move interpolation demo (utility — not used in production)
    partial = arm.calculate_partial_move(*target, percentage=0.80)
    print(f"80 %% move steps: {partial}")
    print(f"80 %% move JSON : {json.dumps(partial)}\n")

    final = arm.calculate_partial_move(*target, percentage=1.0)
    print(f"100 %% move steps: {final}")
    print(f"100 %% move JSON : {json.dumps(final)}")
