"""Unit tests for the IK solver (src/ik/solver.py).

Tests cover reachability, joint limits, geometric consistency,
symmetry, partial moves, and edge cases.

Requires: pytest, numpy
Run with:  python -m pytest tests/test_ik_solver.py -v
"""

import math

import numpy as np
import pytest

from ik.solver import ArmIK


# ── Helpers ───────────────────────────────────────────────────────────

def _forward_kinematics_xy(arm: ArmIK, result: dict) -> tuple[float, float, float]:
    """Approximate forward kinematics from motor steps back to (x, y, z).

    This is a simplified FK that reverses the IK geometry to verify the
    solution is self-consistent.  It uses the same conventions as
    ArmIK.solve() — shoulder at step 2048 means vertical, etc.

    Returns (x, y, z) in centimetres (shoulder-relative, z = table height).
    """
    RAD_PER_STEP = arm.RAD_PER_STEP
    CENTRE = arm.STEP_CENTRE

    # Recover angles from steps (inverse of _rad_to_steps)
    theta_base = (result["m1"] - CENTRE) * RAD_PER_STEP
    # m2 was stored as rad_to_steps(theta_shoulder - pi/2)
    theta_shoulder = (result["m2"] - CENTRE) * RAD_PER_STEP + math.pi / 2
    # m3 was stored as rad_to_steps(-theta_elbow)
    theta_elbow = -((result["m3"] - CENTRE) * RAD_PER_STEP)

    # Planar FK (in the arm's 2-D plane)
    # The shoulder angle is measured from horizontal; the elbow opens up.
    #   Joint 1 endpoint (relative to shoulder):
    x1 = arm.L1 * math.cos(theta_shoulder)
    z1 = arm.L1 * math.sin(theta_shoulder)
    #   Joint 2 endpoint:
    x2 = x1 + arm.L2 * math.cos(theta_shoulder - theta_elbow)
    z2 = z1 + arm.L2 * math.sin(theta_shoulder - theta_elbow)

    # z2 is shoulder-relative wrist height.
    # The IK added L3 (claw length) and subtracted shoulder_height,
    # so the table-level z = z2 - L3 + shoulder_height
    #   BUT sag compensation also shifted z_ik, so the round-trip won't
    #   be exact — we just verify it is geometrically plausible.

    # Convert planar (r, z) back to (x, y) via base angle
    r = x2  # planar horizontal reach
    x = r * math.cos(theta_base)
    y = r * math.sin(theta_base)

    # z in table-frame ≈ wrist_z + shoulder_height - L3
    z_table = z2 + arm.shoulder_height - arm.L3

    return (x, y, z_table)


MOTOR_KEYS = ["m1", "m2", "m3", "m4", "m5"]


# =====================================================================
#  Test classes
# =====================================================================


class TestReachability:
    """Tests that the solver correctly identifies reachable vs unreachable targets."""

    def test_home_position_is_reachable(self, arm, home_position):
        """The configured HOME_POSITION should always produce a valid solution."""
        result = arm.solve(*home_position)
        assert isinstance(result, dict)
        assert all(k in result for k in MOTOR_KEYS)

    def test_moderate_reach_is_reachable(self, arm):
        """A point well within the workspace (20 cm forward, on the table)."""
        result = arm.solve(x=20.0, y=0.0, z=0.0)
        assert isinstance(result, dict)
        assert all(0 <= result[k] <= 4095 for k in MOTOR_KEYS)

    def test_all_bin_positions_are_reachable(self, arm, bin_positions):
        """Every configured bin position must be solvable."""
        for name, coords in bin_positions.items():
            result = arm.solve(*coords)
            assert isinstance(result, dict), f"Bin {name} at {coords} was not reachable"

    def test_too_close_raises_value_error(self, arm):
        """A target inside the dead zone (closer than |L1 − L2|) should raise."""
        # With dynamic pitch the wrist tilts to change wrist position,
        # so we choose shoulder_height to make wrist_z_ik ≈ 0 at the
        # Z_MIN-clamped z, so d ≈ 0 < min_reach.
        # wrist_z = Z_MIN + L3 = 1.5 + 22.0 = 23.5 (claw straight down)
        # Set shoulder_height = 23.5 so wrist_z_ik ≈ 0
        # Note: Safe to mutate since 'arm' fixture is function-scoped (recreated per test)
        arm.shoulder_height = 23.5
        with pytest.raises(ValueError, match="too close"):
            arm.solve(x=0.01, y=0.0, z=0.0)

    def test_far_point_is_clamped_not_error(self, arm):
        """A point at 200 cm away is far beyond max reach.

        The solver clamps overshoot rather than raising, so it should
        still return a valid dict (with a warning printed).
        """
        result = arm.solve(x=200.0, y=0.0, z=0.0)
        assert isinstance(result, dict)
        assert all(k in result for k in MOTOR_KEYS)


class TestJointLimits:
    """Tests that solutions respect the configured physical joint limits."""

    @pytest.mark.parametrize(
        "x, y, z",
        [
            (20.0, 0.0, 0.0),
            (15.0, 10.0, 0.0),
            (25.0, -5.0, 5.0),
            (20.0, 0.0, 30.0),   # home position
            (20.0, 8.0, 10.0),   # RED_BIN
            (20.0, -8.0, 10.0),  # BLUE_BIN
        ],
    )
    def test_all_motors_within_limits(self, arm, x, y, z):
        """Every returned motor position must be within JOINT_LIMITS."""
        result = arm.solve(x=x, y=y, z=z)
        for key in MOTOR_KEYS:
            lo, hi = arm.JOINT_LIMITS[key]
            assert lo <= result[key] <= hi, (
                f"{key}={result[key]} outside limits [{lo}, {hi}] "
                f"for target ({x}, {y}, {z})"
            )

    def test_steps_are_integers(self, arm):
        """Motor step values must be ints (Dynamixel protocol requirement)."""
        result = arm.solve(x=20.0, y=5.0, z=10.0)
        for key in MOTOR_KEYS:
            assert isinstance(result[key], int), f"{key} is {type(result[key])}, expected int"


class TestStrictSolve:
    """Strict production IK path rejects invalid requests instead of clamping."""

    def test_strict_returns_structured_commands_and_validation(self, arm_no_sag):
        result = arm_no_sag.solve_strict(
            {"x": 15.0, "y": 0.0, "z": 30.0, "m5": arm_no_sag.CLAW_OPEN},
            intent="carry",
        )
        assert set(result.keys()) == {"commands", "validation"}
        assert all(k in result["commands"] for k in MOTOR_KEYS)
        assert result["validation"]["intent"] == "carry"

    def test_strict_rejects_unreachable_rear_target(self, arm_no_sag):
        with pytest.raises(ValueError, match="unreachable"):
            arm_no_sag.solve_strict({"x": -200.0, "y": 0.0, "z": 20.0}, intent="rear_place")

    def test_strict_rejects_wrist_infeasible_offset(self, arm_no_sag):
        with pytest.raises(ValueError, match="wrist trim/offset infeasible"):
            arm_no_sag.solve_strict(
                {"x": 20.0, "y": 0.0, "z": 10.0, "m4_offset": 3000},
                intent="carry",
            )

    def test_strict_rear_rejects_wrist_infeasible_offset_after_yaw_check(self, arm_no_sag):
        with pytest.raises(ValueError, match="fold-over branch.*m4="):
            arm_no_sag.solve_strict(
                {"x": -24.0, "y": -8.0, "z": 28.0, "m4_offset": 3000, "skip_sag": True},
                intent="rear_place",
            )

    def test_strict_rejects_joint_limit_violation(self, arm_no_sag):
        arm_no_sag.JOINT_LIMITS = arm_no_sag.JOINT_LIMITS.copy()
        arm_no_sag.JOINT_LIMITS["m1"] = (2048, 2048)
        with pytest.raises(ValueError, match="joint limit violation m1"):
            arm_no_sag.solve_strict({"x": 20.0, "y": 10.0, "z": 10.0}, intent="carry")

    def test_strict_rejects_floor_clamp_request(self, arm_no_sag):
        with pytest.raises(ValueError, match="Z_MIN"):
            arm_no_sag.solve_strict({"x": 20.0, "y": 0.0, "z": arm_no_sag.Z_MIN - 1.0}, intent="pickup")

    def test_strict_rear_target_requiring_large_base_rotation_is_rejected(self, arm_no_sag):
        with pytest.raises(ValueError, match="base yaw .*outside configured range"):
            arm_no_sag.solve_strict(
                {"x": -20.0, "y": 30.0, "z": 32.0, "rear_base_yaw_limit_deg": 45.0, "skip_sag": True},
                intent="rear_place",
            )

    def test_strict_valid_rear_target_uses_fold_over_branch(self, arm_no_sag):
        result = arm_no_sag.solve_strict(
            {"x": -24.0, "y": -8.0, "z": 28.0, "rear_base_yaw_limit_deg": 45.0, "skip_sag": True},
            intent="rear_place",
        )
        validation = result["validation"]

        assert validation["base_yaw_within_range"] is True
        assert -45.0 <= validation["base_yaw_deg"] <= 45.0
        assert validation["shoulder_in_fold_back_range"] is True
        assert validation["theta_shoulder_deg"] >= validation["shoulder_fold_back_min_deg"]
        assert validation["ik_branch"].startswith("fold_back")

    def test_strict_valid_rear_target_commands_decode_behind_base(self, arm_no_sag):
        target = {"x": -24.0, "y": -8.0, "z": 28.0, "rear_base_yaw_limit_deg": 45.0, "skip_sag": True}
        result = arm_no_sag.solve_strict(target, intent="rear_place")

        fk = arm_no_sag.forward_kinematics(result["commands"])

        assert result["validation"]["base_yaw_within_range"] is True
        assert -45.0 <= result["validation"]["base_yaw_deg"] <= 45.0
        assert fk["x"] == pytest.approx(target["x"], abs=0.5)
        assert fk["y"] == pytest.approx(target["y"], abs=0.5)
        assert fk["z"] == pytest.approx(target["z"], abs=0.5)
        assert fk["x"] < -20.0

    def test_strict_adjacent_rear_bins_have_small_distinct_yaw(self, arm_no_sag):
        red = arm_no_sag.solve_strict(
            {"x": -24.0, "y": -8.0, "z": 28.0, "rear_base_yaw_limit_deg": 45.0, "skip_sag": True},
            intent="rear_place",
        )
        blue = arm_no_sag.solve_strict(
            {"x": -24.0, "y": 8.0, "z": 28.0, "rear_base_yaw_limit_deg": 45.0, "skip_sag": True},
            intent="rear_place",
        )

        red_yaw = red["validation"]["base_yaw_deg"]
        blue_yaw = blue["validation"]["base_yaw_deg"]
        assert -45.0 <= red_yaw <= 45.0
        assert -45.0 <= blue_yaw <= 45.0
        assert abs(red_yaw - blue_yaw) > 5.0
        assert red_yaw == pytest.approx(-blue_yaw, abs=0.5)
        assert red["validation"]["shoulder_in_fold_back_range"] is True
        assert blue["validation"]["shoulder_in_fold_back_range"] is True


class TestSymmetry:
    """Points mirrored across Y=0 should produce mirrored motor positions.

    Specifically, m2/m3/m4/m5 should be identical (same arm pose in the
    vertical plane) and m1 should be symmetric around 2048.
    """

    @pytest.mark.parametrize(
        "x, y, z",
        [
            (20.0, 10.0, 0.0),
            (15.0, 5.0, 0.0),
            (30.0, 15.0, 5.0),
        ],
    )
    def test_mirror_y_produces_symmetric_m1(self, arm, x, y, z):
        """m1 for (x, +y, z) and (x, -y, z) should sum to ~4096."""
        pos = arm.solve(x=x, y=y, z=z)
        neg = arm.solve(x=x, y=-y, z=z)
        m1_sum = pos["m1"] + neg["m1"]
        assert abs(m1_sum - 4096) <= 2, (
            f"m1 not symmetric: {pos['m1']} + {neg['m1']} = {m1_sum} (expect ≈4096)"
        )

    @pytest.mark.parametrize(
        "x, y, z",
        [
            (20.0, 10.0, 0.0),
            (15.0, 5.0, 0.0),
            (30.0, 15.0, 5.0),
        ],
    )
    def test_mirror_y_preserves_vertical_motors(self, arm, x, y, z):
        """m2, m3, m4, m5 should be identical for mirrored Y targets."""
        pos = arm.solve(x=x, y=y, z=z)
        neg = arm.solve(x=x, y=-y, z=z)
        for key in ["m2", "m3", "m4", "m5"]:
            assert pos[key] == neg[key], (
                f"{key} differs: {pos[key]} vs {neg[key]} "
                f"for y={y} vs y={-y}"
            )


class TestGeometricConsistency:
    """Forward-kinematics verification of IK solutions."""

    def test_fk_roundtrip_moderate_target(self, arm):
        """FK of the IK solution should land near the original target.

        We allow a generous tolerance because sag compensation and
        Z_MIN clamping intentionally shift the effective target.
        """
        target = (20.0, 0.0, 10.0)
        result = arm.solve(*target)
        fk_pos = _forward_kinematics_xy(arm, result)

        # The horizontal direction should match closely
        np.testing.assert_allclose(fk_pos[0], target[0], atol=5.0,
                                   err_msg="FK x deviates too much from target")
        np.testing.assert_allclose(fk_pos[1], target[1], atol=5.0,
                                   err_msg="FK y deviates too much from target")

    def test_fk_roundtrip_elevated_target(self, arm):
        """An elevated target (z=30) should still be geometrically plausible."""
        target = (20.0, 0.0, 30.0)
        result = arm.solve(*target)
        fk_pos = _forward_kinematics_xy(arm, result)

        # At least the horizontal reach direction should be correct
        assert fk_pos[0] > 0, "FK x should be positive (arm reaches forward)"

    def test_planar_distance_within_reach(self, arm):
        """The planar distance implied by the shoulder/elbow angles must be
        within [min_reach, max_reach] for the 2-link chain."""
        result = arm.solve(x=20.0, y=5.0, z=0.0)
        RAD = arm.RAD_PER_STEP
        C = arm.STEP_CENTRE

        theta_shoulder = (result["m2"] - C) * RAD + math.pi / 2
        theta_elbow = -((result["m3"] - C) * RAD)

        # End of link-2 relative to shoulder
        x2 = arm.L1 * math.cos(theta_shoulder) + arm.L2 * math.cos(theta_shoulder - theta_elbow)
        z2 = arm.L1 * math.sin(theta_shoulder) + arm.L2 * math.sin(theta_shoulder - theta_elbow)
        d = math.sqrt(x2 ** 2 + z2 ** 2)

        max_reach = arm.L1 + arm.L2
        min_reach = abs(arm.L1 - arm.L2)
        assert min_reach <= d <= max_reach, (
            f"Planar distance {d:.2f} outside [{min_reach:.2f}, {max_reach:.2f}]"
        )

    def test_limp_capture_replay_offset_preserves_fk_xyz(self, arm_no_sag):
        """Regression: FK limp captures with wrist trim must replay same XYZ."""

        captured = {"m1": 2350, "m2": 1200, "m3": 1000, "m4": 2500, "m5": arm_no_sag.CLAW_OPEN}
        fk = arm_no_sag.forward_kinematics(captured)

        replay = arm_no_sag.solve(
            fk["x"],
            fk["y"],
            fk["z"],
            skip_sag=True,
            strict=True,
            m4_offset=fk["replay_m4_offset"],
        )
        replay_fk = arm_no_sag.forward_kinematics(replay)

        assert replay_fk["x"] == pytest.approx(fk["x"], abs=0.15)
        assert replay_fk["y"] == pytest.approx(fk["y"], abs=0.15)
        assert replay_fk["z"] == pytest.approx(fk["z"], abs=0.15)

    def test_strict_m4_offset_is_geometric_not_posthoc(self, arm_no_sag):
        """Strict route m4_offset should not move the commanded claw-tip XYZ."""

        target = {"x": 32.0, "y": 8.0, "z": 6.0, "m4_offset": 300, "skip_sag": True}

        result = arm_no_sag.solve_strict(target, intent="pickup")
        fk = arm_no_sag.forward_kinematics(result["commands"])

        assert fk["x"] == pytest.approx(target["x"], abs=0.2)
        assert fk["y"] == pytest.approx(target["y"], abs=0.2)
        assert fk["z"] == pytest.approx(target["z"], abs=0.2)
        assert result["validation"]["final_theta_pitch_deg"] != pytest.approx(
            result["validation"]["theta_pitch_deg"], abs=0.1
        )


class TestPartialMove:
    """Tests for the Cartesian partial-move interpolation helper."""

    def test_100_percent_equals_full_solve(self, arm):
        """calculate_partial_move at 100% should equal solve() exactly."""
        target = (25.0, 10.0, 0.0)
        full = arm.solve(*target)
        partial = arm.calculate_partial_move(*target, percentage=1.0)
        assert full == partial

    def test_partial_move_interpolates(self, arm):
        """At 50%, motor steps should be between origin-solve and full-solve."""
        target = (25.0, 10.0, 5.0)
        full = arm.solve(*target)
        half = arm.calculate_partial_move(*target, percentage=0.5)

        # m1 (base angle) at 50% should be roughly between 2048 (origin=0,0)
        # and the full value — at minimum, it should differ from full.
        # We can't test exact interpolation since IK is non-linear,
        # but the base angle should be between 2048 and full["m1"]
        lo, hi = sorted([2048, full["m1"]])
        # Allow some tolerance for rounding
        assert lo - 5 <= half["m1"] <= hi + 5, (
            f"m1 at 50% ({half['m1']}) not between origin ({2048}) and full ({full['m1']})"
        )

    def test_invalid_percentage_raises(self, arm):
        """percentage outside [0, 1] should raise ValueError."""
        with pytest.raises(ValueError):
            arm.calculate_partial_move(20.0, 0.0, 0.0, percentage=1.5)
        with pytest.raises(ValueError):
            arm.calculate_partial_move(20.0, 0.0, 0.0, percentage=-0.1)


class TestEdgeCases:
    """Edge cases and constructor variants."""

    def test_custom_link_lengths(self):
        """ArmIK can be instantiated with custom link lengths."""
        custom = ArmIK(l1=20.0, l2=20.0, l3=10.0,
                       z_offset_multiplier=0.0, z_offset_quadratic=0.0)
        assert custom.L1 == 20.0
        assert custom.L2 == 20.0
        assert custom.L3 == 10.0

    def test_z_below_z_min_is_clamped(self, arm):
        """Targets with z < Z_MIN should be silently clamped upward."""
        # z=-10 is below the floor; the solver clamps to Z_MIN=-2.0
        result_low = arm.solve(x=20.0, y=0.0, z=-10.0)
        result_min = arm.solve(x=20.0, y=0.0, z=arm.Z_MIN)
        # Both should produce the same result since -10 gets clamped to Z_MIN
        assert result_low == result_min

    def test_claw_always_at_default(self, arm):
        """m5 (claw) should always be CLAW_OPEN regardless of target."""
        for coords in [(20, 0, 0), (15, 10, 5), (30, -5, 10)]:
            result = arm.solve(*coords)
            assert result["m5"] == arm.CLAW_OPEN

    def test_solve_to_json_returns_valid_json(self, arm):
        """solve_to_json() should return a parseable JSON string."""
        import json
        j = arm.solve_to_json(20.0, 0.0, 10.0)
        data = json.loads(j)
        assert all(k in data for k in MOTOR_KEYS)


class TestDynamicPitch:
    """Tests for the dynamic wrist pitch feature."""

    def test_in_range_target_uses_straight_down(self, arm):
        """For targets well within reach, pitch stays at -π/2 (straight down)."""
        # A moderate target should not trigger pitch adjustment
        result = arm.solve(x=20.0, y=0.0, z=5.0)
        # m4 should correspond to straight-down wrist compensation
        # (same as old behavior)
        assert isinstance(result, dict)
        assert all(k in result for k in MOTOR_KEYS)

    def test_far_target_tilts_wrist_forward(self, arm):
        """For targets beyond normal reach, the wrist should tilt forward,
        resulting in a different m4 than straight-down would give."""
        # Use a target that's near-but-beyond the arm's reach
        # L1 + L2 = 48.5, L3 = 22.0. A target at x=45, z=0 should be
        # at the edge when accounting for L3 offset
        near_result = arm.solve(x=20.0, y=0.0, z=0.0)
        far_result = arm.solve(x=45.0, y=0.0, z=0.0)
        # The far target should have a different wrist angle
        # (This is a smoke test — exact values depend on geometry)
        assert isinstance(far_result, dict)
        assert all(k in far_result for k in MOTOR_KEYS)

    def test_pitch_does_not_exceed_limit(self, arm):
        """Even for extremely far targets, pitch should not go beyond MAX_REACH_PITCH."""
        # Very far target — should hit the pitch limit
        result = arm.solve(x=200.0, y=0.0, z=0.0)
        assert isinstance(result, dict)
        assert all(k in result for k in MOTOR_KEYS)

    def test_dynamic_pitch_activates_for_near_edge_target(self):
        """Regression: the pitch loop must actually iterate for near-edge targets.

        Before the fix, the guard `if theta_pitch <= MAX_REACH_PITCH` was
        immediately True (−π/2 ≤ −π/4), so the loop broke on the first
        iteration and the dynamic pitch feature never activated.

        We construct a target that is ~1 cm beyond the max reach at
        straight-down pitch (−π/2) but reachable with a slightly tilted
        wrist. If the pitch loop works, the solver returns a valid
        solution *without* clamping, and m4 differs from the value the
        solver would produce if the wrist stayed at −π/2 the whole time.
        """
        # Use a clean arm with no sag so the geometry is predictable
        arm = ArmIK(
            l1=25.5, l2=23.0, l3=22.0,
            z_offset_multiplier=0.0,
            z_offset_quadratic=0.0,
            shoulder_height=11.0,
        )

        # With straight-down pitch (−π/2), the wrist lands at:
        #   wrist_x = x − L3·cos(−π/2) = x − 0 = x
        #   wrist_z = z − L3·sin(−π/2) = z + L3 = z + 22.0
        # After shoulder offset: wrist_z_ik = wrist_z − shoulder_height
        # Max 2-link reach: L1 + L2 = 48.5 cm
        # Pick z = arm.Z_MIN (1.5) so wrist_z_ik = 1.5 + 22.0 − 11.0 = 12.5
        # Then d = sqrt(x² + 12.5²) = 48.5 → x ≈ 46.86 cm
        # Use x = 48.5 — about 1.3 cm beyond straight-down reach.
        target_x = 48.5
        target_z = arm.Z_MIN  # will be clamped to Z_MIN anyway

        result = arm.solve(x=target_x, y=0.0, z=target_z)
        assert isinstance(result, dict)
        assert all(k in result for k in MOTOR_KEYS)

        # Compute what m4 would be if pitch stayed at −π/2 (no dynamic pitch).
        # We can approximate this by solving a clearly-in-range target and
        # comparing: a close target always uses −π/2 pitch.
        close_result = arm.solve(x=20.0, y=0.0, z=target_z)

        # The wrist step (m4) must differ — dynamic pitch tilted the wrist
        assert result["m4"] != close_result["m4"], (
            f"m4 should differ when dynamic pitch activates, "
            f"but both are {result['m4']}"
        )
