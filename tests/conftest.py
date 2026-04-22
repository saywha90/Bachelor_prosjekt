"""Shared pytest fixtures for the Autonomia test suite."""

import pytest

# pytest automatically adds src/ to sys.path via pyproject.toml:
#   [tool.pytest.ini_options]
#   pythonpath = ["src"]
from ik.solver import ArmIK
from config.arm import (
    BINS,
    HOME_POSITION,
    GRAB_HEIGHT,
    APPROACH_HEIGHT,
    CLEARANCE_HEIGHT,
)


@pytest.fixture
def arm():
    """Return an ArmIK instance with production link lengths (no sag file)."""
    # Explicitly set sag parameters to defaults so the fixture is
    # deterministic even if a sag_calibration.json file exists on disk.
    return ArmIK(
        z_offset_multiplier=0.04,
        z_offset_quadratic=0.0,
        sag_model="linear",
    )


@pytest.fixture
def arm_no_sag():
    """Return an ArmIK instance with sag compensation disabled."""
    return ArmIK(
        z_offset_multiplier=0.0,
        z_offset_quadratic=0.0,
        sag_model="linear",
    )


@pytest.fixture
def home_position():
    """Return the configured home/rest position (x, y, z) in cm."""
    return HOME_POSITION


@pytest.fixture
def bin_positions():
    """Return the dict of bin name → (x, y, z) coordinates."""
    return BINS
