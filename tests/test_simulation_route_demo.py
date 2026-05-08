"""Tests for rear-placement route simulation prevalidation helpers."""

import json

import pytest

from config.arm import RouteCalibrationError, load_transport_route_calibration
from ik.solver import ArmIK
from simulation.route_demo import (
    SAMPLE_ROUTE_CALIBRATION,
    build_air_pick_scan_demo_plan,
    build_rear_placement_demo_plan,
    main as route_demo_main,
    route_overlay_points,
    scan_look_again_command,
)
from simulation.visualizer import (
    SIMULATION_REAR_BINS,
    forward_kinematics as visualizer_forward_kinematics,
)


def _deterministic_arm():
    return ArmIK(
        z_offset_multiplier=0.0,
        z_offset_quadratic=0.0,
        z_offset_constant=0.0,
        sag_model="linear",
    )


def _write_json(path, data):
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def test_sample_route_demo_plan_prevalidates_ordered_waypoints():
    plan = build_rear_placement_demo_plan(
        _deterministic_arm(),
        SAMPLE_ROUTE_CALIBRATION,
        destination="RED_BIN",
    )

    names = [waypoint.name for waypoint in plan]
    assert names[:8] == [
        "pickup-clearance",
        "pickup",
        "pickup-gripped",
        "pickup-clearance-retreat",
        "front-neutral",
        "rear-transfer",
        "bin-drop",
        "bin-drop-release",
    ]
    assert names[-3:] == [
        "rear-return-lift",
        "front-neutral-retreat",
        "pickup-clearance-home",
    ]
    assert all(set(waypoint.commands) == {"m1", "m2", "m3", "m4", "m5"} for waypoint in plan)
    for waypoint in plan:
        if waypoint.intent == "rear_place":
            validation = waypoint.validation
            assert validation["base_yaw_within_range"] is True
            assert -45.0 <= validation["base_yaw_deg"] <= 45.0
            assert validation["shoulder_in_fold_back_range"] is True


def test_sample_route_and_visual_layout_have_only_two_rear_bins():
    route_cal = load_transport_route_calibration(SAMPLE_ROUTE_CALIBRATION, require_route_schema=True)

    assert set(route_cal.bins) == {"RED_BIN", "BLUE_BIN"}
    assert set(SIMULATION_REAR_BINS) == {"RED_BIN", "BLUE_BIN"}
    assert "REJECT_BIN" not in route_cal.bins
    assert "REJECT_BIN" not in SIMULATION_REAR_BINS


def test_sample_route_bins_are_behind_robot_and_beside_each_other():
    route_cal = load_transport_route_calibration(SAMPLE_ROUTE_CALIBRATION, require_route_schema=True)
    red_drop = route_cal.bins["RED_BIN"].drop
    blue_drop = route_cal.bins["BLUE_BIN"].drop
    red_visual = SIMULATION_REAR_BINS["RED_BIN"]
    blue_visual = SIMULATION_REAR_BINS["BLUE_BIN"]

    assert red_drop.x < 0.0
    assert blue_drop.x < 0.0
    assert red_drop.x == pytest.approx(blue_drop.x)
    assert red_drop.y < 0.0 < blue_drop.y
    assert abs(red_drop.y) == pytest.approx(abs(blue_drop.y))

    assert red_visual[0] < 0.0
    assert blue_visual[0] < 0.0
    assert red_visual[0] == pytest.approx(blue_visual[0])
    assert red_visual[1] < 0.0 < blue_visual[1]


def test_route_overlay_points_keep_visual_corridor_names():
    plan = build_rear_placement_demo_plan(
        _deterministic_arm(),
        SAMPLE_ROUTE_CALIBRATION,
        destination="BLUE_BIN",
    )

    overlay_names = [name for name, _pose in route_overlay_points(plan)]

    assert overlay_names == [
        "pickup-clearance",
        "pickup",
        "pickup-clearance-retreat",
        "front-neutral",
        "rear-transfer",
        "bin-drop",
        "rear-return-lift",
        "front-neutral-retreat",
        "pickup-clearance-home",
    ]


def test_sample_route_rear_waypoints_render_behind_base_with_small_yaw():
    plan = build_rear_placement_demo_plan(
        _deterministic_arm(),
        SAMPLE_ROUTE_CALIBRATION,
        destination="RED_BIN",
    )

    rear_waypoints = [waypoint for waypoint in plan if waypoint.intent == "rear_place"]

    assert rear_waypoints
    for waypoint in rear_waypoints:
        joints = visualizer_forward_kinematics(**waypoint.commands)
        tip = joints[-1]
        min_x = min(point[0] for point in joints)

        assert waypoint.validation["base_yaw_within_range"] is True
        assert -45.0 <= waypoint.validation["base_yaw_deg"] <= 45.0
        assert tip[0] == pytest.approx(waypoint.pose["x"], abs=0.5)
        assert tip[1] == pytest.approx(waypoint.pose["y"], abs=0.5)
        assert tip[2] == pytest.approx(waypoint.pose["z"], abs=0.5)
        assert tip[0] < 0.0
        assert min_x < 0.0


def test_red_and_blue_routes_have_fold_over_with_small_distinct_yaw():
    red_plan = build_rear_placement_demo_plan(
        _deterministic_arm(),
        SAMPLE_ROUTE_CALIBRATION,
        destination="RED_BIN",
    )
    blue_plan = build_rear_placement_demo_plan(
        _deterministic_arm(),
        SAMPLE_ROUTE_CALIBRATION,
        destination="BLUE_BIN",
    )

    red_drop = next(waypoint for waypoint in red_plan if waypoint.name == "bin-drop")
    blue_drop = next(waypoint for waypoint in blue_plan if waypoint.name == "bin-drop")
    red_yaw = red_drop.validation["base_yaw_deg"]
    blue_yaw = blue_drop.validation["base_yaw_deg"]

    assert red_drop.validation["shoulder_in_fold_back_range"] is True
    assert blue_drop.validation["shoulder_in_fold_back_range"] is True
    assert abs(red_yaw) < 45.0
    assert abs(blue_yaw) < 45.0
    assert red_yaw == pytest.approx(-blue_yaw, abs=0.5)
    assert abs(red_yaw - blue_yaw) > 5.0


def test_air_pick_demo_returns_to_scan_and_does_not_use_reject_bin(capsys):
    plan = build_air_pick_scan_demo_plan(_deterministic_arm())

    assert [waypoint.name for waypoint in plan] == [
        "pickup-clearance",
        "pickup",
        "pickup-no-grip-close",
        "pickup-clearance-retreat",
    ]
    assert all("reject" not in waypoint.name.lower() for waypoint in plan)
    assert scan_look_again_command()["m1"] == 2048

    rc = route_demo_main([
        "--calibration",
        str(SAMPLE_ROUTE_CALIBRATION),
        "--air-pick",
        "--no-gui",
    ])
    output = capsys.readouterr().out.lower()

    assert rc == 0
    assert "no object was gripped" in output
    assert "scan/look-again" in output
    assert "reject" not in output


def test_route_demo_fails_closed_for_legacy_calibration(tmp_path):
    legacy_path = _write_json(
        tmp_path / "legacy_bin_calibration.json",
        {"bins": {"RED_BIN": {"x": -40.0, "y": -8.0, "z": 30.0}}},
    )

    with pytest.raises(RouteCalibrationError, match="requires route schema"):
        build_rear_placement_demo_plan(_deterministic_arm(), legacy_path, destination="RED_BIN")


def test_route_demo_reports_strict_waypoint_failure(tmp_path):
    invalid_path = _write_json(
        tmp_path / "invalid_route_calibration.json",
        {
            "schema_version": 2,
            "shared_waypoints": {
                "front_neutral": {"x": 22.0, "y": 0.0, "z": 30.0, "skip_sag": True},
                "rear_transfer": {"x": -18.0, "y": -0.5, "z": 38.0, "skip_sag": True},
            },
            "bins": {
                "RED_BIN": {
                    "drop": {"x": -999.0, "y": -8.0, "z": 28.0, "skip_sag": True},
                }
            },
        },
    )

    with pytest.raises(ValueError, match="bin-drop"):
        build_rear_placement_demo_plan(_deterministic_arm(), invalid_path, destination="RED_BIN")
