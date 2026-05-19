"""Tests for limp-to-WASD XY safety checks in touch calibration."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from ik.solver import ArmIK


def _load_touch_calibration_module():
    path = Path(__file__).resolve().parents[1] / "src" / "calibration" / "09_touch_calibration.py"
    spec = importlib.util.spec_from_file_location("touch_calibration_09", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


tc = _load_touch_calibration_module()


def test_limp_xy_validation_rejects_strict_joint_limit_clamp(capsys):
    arm = ArmIK(z_offset_multiplier=0.0, z_offset_quadratic=0.0, sag_model="linear")

    solution = tc._solve_ik_for_command(
        arm,
        x=10.0,
        y=10.0,
        z=20.0,
        validate_xy=True,
        context="limp fine-tune approach",
    )

    assert solution is None
    out = capsys.readouterr().out
    assert "Limp-to-WASD safety stop" in out
    assert "current clearance/start height" in out
    assert "wrong XY" in out


def test_limp_xy_validation_rejects_fk_xy_mismatch_without_sending(capsys):
    arm = ArmIK(z_offset_multiplier=0.0, z_offset_quadratic=0.0, sag_model="linear")
    mismatched_solution = arm.solve(x=20.0, y=10.0, z=20.0, skip_sag=True, strict=True)

    ok = tc._validate_ik_fk_xy(
        arm,
        mismatched_solution,
        x=10.0,
        y=10.0,
        z=20.0,
        xy_tolerance_cm=1.0,
        context="limp fine-tune descent",
    )

    assert ok is False
    out = capsys.readouterr().out
    assert "XY error" in out
    assert "limit 1.0 cm" in out
    assert "Aborting before WASD refinement" in out


def test_limp_xy_validation_accepts_matching_solution():
    arm = ArmIK(z_offset_multiplier=0.0, z_offset_quadratic=0.0, sag_model="linear")
    solution = arm.solve(x=20.0, y=10.0, z=20.0, skip_sag=True, strict=True)

    assert tc._validate_ik_fk_xy(arm, solution, x=20.0, y=10.0, z=20.0, xy_tolerance_cm=1.0)
