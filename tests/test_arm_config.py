"""Tests for route-aware arm calibration loading."""

import json

import pytest

from config.arm import (
    DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG,
    RouteCalibrationError,
    load_transport_route_calibration,
)


def _write_json(path, data):
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def test_legacy_bin_schema_still_loads(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {
            "bins": {
                "RED_BIN": {"x": -40.0, "y": -8.0, "z": 30.0, "m4_offset": 100},
                "BLUE_BIN": {"x": -40.0, "y": 8.0, "z": 30.0},
            }
        },
    )

    route = load_transport_route_calibration(path)

    assert route.source_schema == "legacy"
    assert route.schema_version == 1
    assert route.rear_base_yaw_limit_deg == DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG
    assert route.bins["RED_BIN"].drop.x == -40.0
    assert route.bins["RED_BIN"].drop.m4_offset == 100


def test_route_schema_loads_and_is_preferred(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {
            "schema_version": 2,
            "rear_base_yaw_limit_deg": 45.0,
            "shared_waypoints": {
                "front_neutral": {"x": 20.0, "y": 0.0, "z": 25.0},
                "rear_transfer": {"x": -20.0, "y": 0.0, "z": 35.0, "m4_offset": -10},
            },
            "bins": {
                "RED_BIN": {
                    "x": 99.0,
                    "y": 99.0,
                    "z": 99.0,
                    "approach": {"x": -35.0, "y": -10.0, "z": 35.0},
                    "drop": {"x": -42.0, "y": -10.0, "z": 28.0, "m4_offset": 50},
                }
            },
        },
    )

    route = load_transport_route_calibration(path, require_route_schema=True)

    assert route.source_schema == "route"
    assert route.schema_version == 2
    assert route.rear_base_yaw_limit_deg == 45.0
    assert route.shared_waypoints["rear_transfer"].m4_offset == -10
    assert route.bins["RED_BIN"].approach.x == -35.0
    assert route.bins["RED_BIN"].drop.m4_offset == 50


def test_route_schema_defaults_rear_base_yaw_limit(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {
            "schema_version": 2,
            "shared_waypoints": {
                "front_neutral": {"x": 20.0, "y": 0.0, "z": 25.0},
                "rear_transfer": {"x": -20.0, "y": 0.0, "z": 35.0},
            },
            "bins": {
                "RED_BIN": {
                    "approach": {"x": -20.0, "y": -8.0, "z": 35.0},
                    "drop": {"x": -24.0, "y": -8.0, "z": 28.0},
                }
            },
        },
    )

    route = load_transport_route_calibration(path, require_route_schema=True)

    assert route.rear_base_yaw_limit_deg == DEFAULT_REAR_ROUTE_BASE_YAW_LIMIT_DEG


def test_route_schema_rejects_rear_waypoint_outside_base_yaw_limit(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {
            "schema_version": 2,
            "rear_base_yaw_limit_deg": 45.0,
            "shared_waypoints": {
                "front_neutral": {"x": 20.0, "y": 0.0, "z": 25.0},
                "rear_transfer": {"x": -20.0, "y": 0.0, "z": 35.0},
            },
            "bins": {
                "RED_BIN": {
                    "approach": {"x": -20.0, "y": 30.0, "z": 35.0},
                    "drop": {"x": -24.0, "y": 8.0, "z": 28.0},
                }
            },
        },
    )

    with pytest.raises(RouteCalibrationError, match="base yaw .*outside"):
        load_transport_route_calibration(path, require_route_schema=True)


def test_route_schema_missing_shared_waypoint_fails_clearly(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {
            "schema_version": 2,
            "shared_waypoints": {
                "front_neutral": {"x": 20.0, "y": 0.0, "z": 25.0},
            },
            "bins": {
                "RED_BIN": {
                    "approach": {"x": -35.0, "y": -10.0, "z": 35.0},
                    "drop": {"x": -42.0, "y": -10.0, "z": 28.0},
                }
            },
        },
    )

    with pytest.raises(RouteCalibrationError, match="shared waypoint.*rear_transfer"):
        load_transport_route_calibration(path, require_route_schema=True)


def test_route_schema_missing_bin_drop_fails_clearly(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {
            "schema_version": 2,
            "shared_waypoints": {
                "front_neutral": {"x": 20.0, "y": 0.0, "z": 25.0},
                "rear_transfer": {"x": -20.0, "y": 0.0, "z": 35.0},
            },
            "bins": {
                "RED_BIN": {
                    "approach": {"x": -35.0, "y": -10.0, "z": 35.0},
                }
            },
        },
    )

    with pytest.raises(RouteCalibrationError, match="RED_BIN.*drop"):
        load_transport_route_calibration(path, require_route_schema=True)


def test_production_requires_route_schema(tmp_path):
    path = _write_json(
        tmp_path / "bin_calibration.json",
        {"bins": {"RED_BIN": {"x": -40.0, "y": -8.0, "z": 30.0}}},
    )

    with pytest.raises(RouteCalibrationError, match="requires route schema"):
        load_transport_route_calibration(path, require_route_schema=True)
